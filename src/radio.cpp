// ============================================================
// RF.Guru LoRa 433 APRS Tracker - Radio abstraction
//
// Identifies the fitted radio over raw SPI, then drives it through
// RadioLib. See include/radio.h for the rationale.
// ============================================================

#include "radio.h"

#include <SPI.h>
#include <RadioLib.h>
#include <hardware/watchdog.h>

#include "pins.h"

// ------------------------------------------------------------
// Fixed on-air parameters
//
// These reproduce exactly what the V1 firmware put on the air with
// sandeepmistry/LoRa, so a V2 board is decoded by the same gateways:
// SF12, 125 kHz, coding rate 4/5, 8-symbol preamble, explicit header,
// CRC on, sync word 0x12. On the SX126x the 8-bit 0x12 expands to the
// 16-bit register value 0x1424, which is the same sync word on air.
// ------------------------------------------------------------
static const float    LORA_BW_KHZ     = 125.0f;
static const uint8_t  LORA_SF         = 12;
static const uint8_t  LORA_CR         = 5;        // 4/5
static const uint8_t  LORA_SYNC_WORD  = 0x12;
static const uint16_t LORA_PREAMBLE   = 8;

// Over-current protection. RadioLib's begin() leaves both families at
// 60 mA, which would clamp the PA at full output. 140 mA is Semtech's
// figure for +20 dBm PA_BOOST and is what the V1 firmware programmed
// (sandeepmistry's setTxPower calls setOCP(140) above +17 dBm).
static const uint8_t LORA_OCP_SX127X_MA = 140;
static const float   LORA_OCP_SX126X_MA = 140.0f;

// ------------------------------------------------------------
// G-NiceRF 1262MiniF27 (V2 boards)
//
// The module is an SX1262 die followed by an internal power amplifier,
// so the chip's output is the drive into that amplifier rather than the
// radiated power - the module turns roughly +22 dBm of die output into
// ~29 dBm at the antenna pad.
//
// The datasheet's "Chip Output Level 0..9" column is an index across
// the die's range, NOT a figure in dBm. Reading it as dBm implies the
// module's gain grows from 9.8 dB at level 0 to 19.7 dB at level 9,
// which no amplifier does; as an index it is ordinary compression.
// Measurement settled it: driving the die to its full +22 dBm produced
// the highest output on a bench sweep and the current draw then matched
// the datasheet's efficiency (~42% measured against ~40% published).
// Clamping to "9 dBm" instead cost about 4 dB.
//
// So paDrive is the die's output power in dBm and wants to be at the
// maximum for rated output. What actually leaves the antenna depends on
// VCC_PA (26.6 dBm at 3.3 V rising to 29.8 dBm at 5.0 V) and on the
// filtering after the module, so the firmware does not try to predict
// it - it reports what it programmed.
static const int8_t MINIF27_MIN_DRIVE = -9;
static const int8_t MINIF27_MAX_DRIVE = 22;

// SPI clock used for both the raw probe and RadioLib.
static const uint32_t LORA_SPI_HZ = 2000000;

// ------------------------------------------------------------
// State
// ------------------------------------------------------------
static SPISettings   loraSpi(LORA_SPI_HZ, MSBFIRST, SPI_MODE0);

// Two Module instances: the SX126x needs its BUSY handshake line, the
// SX127x has no equivalent and must not be given one.
static Module modSx127x(PIN_LORA_CS, RADIOLIB_NC, PIN_LORA_RST, RADIOLIB_NC, SPI, loraSpi);
static Module modSx1262(PIN_LORA_CS, RADIOLIB_NC, PIN_LORA_RST, PIN_LORA_BUSY, SPI, loraSpi);
static Module modSx1268(PIN_LORA_CS, RADIOLIB_NC, PIN_LORA_RST, PIN_LORA_BUSY, SPI, loraSpi);

static SX1276 radioSx127x(&modSx127x);
static SX1262 radioSx1262(&modSx1262);
static SX1268 radioSx1268(&modSx1268);

static PhysicalLayer *phy        = nullptr;
static SX126x        *sx126      = nullptr;   // whichever SX126x variant is fitted
static RadioChip      activeChip = RADIO_CHIP_NONE;
static uint32_t       txDoneMask = 0;
static int8_t         appliedPwr = 0;
static float          tcxoVolts  = 0.0f;
static bool           spiStarted = false;
static bool           modulePa   = false;   // module has its own amplifier
static float          ocpMa      = LORA_OCP_SX126X_MA;   // as configured, for setDrive()
static bool           useLdoReg  = false;   // die regulator: LDO vs DC-DC

// The 16-byte version string at register 0x0320 is the only way to tell
// the SX126x variants apart, and it matters: RadioLib's SX1262 class
// refuses to start on an SX1268 with ERR_CHIP_NOT_FOUND, which looks
// exactly like a wiring fault. SX1261 and SX1262 both report "SX1261".
static char sx126xVersion[8] = "";

// ============================================================
// Raw SPI probe
//
// Runs before RadioLib touches the bus. Read-only apart from a single
// scratch write to the SX126x sync word register, which is restored.
// Emits no RF.
// ============================================================

static void rawBegin() {
    if (spiStarted) return;
    pinMode(PIN_LORA_CS, OUTPUT);
    digitalWrite(PIN_LORA_CS, HIGH);
    pinMode(PIN_LORA_RST, OUTPUT);
    digitalWrite(PIN_LORA_RST, HIGH);
    pinMode(PIN_LORA_BUSY, INPUT);

    SPI.setRX(PIN_SPI_MISO);
    SPI.setTX(PIN_SPI_MOSI);
    SPI.setSCK(PIN_SPI_CLK);
    SPI.begin();
    spiStarted = true;
}

static void rawReset() {
    digitalWrite(PIN_LORA_RST, LOW);
    delay(5);
    digitalWrite(PIN_LORA_RST, HIGH);
    delay(30);
}

// SX126x holds BUSY high while it digests a command.
static void rawWaitBusy(uint32_t timeoutUs = 20000) {
    uint32_t t0 = micros();
    while (digitalRead(PIN_LORA_BUSY)) {
        if ((uint32_t)(micros() - t0) > timeoutUs) return;
    }
}

// SX127x framing: 7-bit address, bit 7 clear selects a read.
static uint8_t rawSx127xRead(uint8_t addr) {
    SPI.beginTransaction(loraSpi);
    digitalWrite(PIN_LORA_CS, LOW);
    SPI.transfer(addr & 0x7F);
    uint8_t v = SPI.transfer(0x00);
    digitalWrite(PIN_LORA_CS, HIGH);
    SPI.endTransaction();
    return v;
}

static void rawSx126xCmd(uint8_t opcode, const uint8_t *data, size_t n) {
    SPI.beginTransaction(loraSpi);
    digitalWrite(PIN_LORA_CS, LOW);
    SPI.transfer(opcode);
    for (size_t i = 0; i < n; i++) SPI.transfer(data[i]);
    digitalWrite(PIN_LORA_CS, HIGH);
    SPI.endTransaction();
    rawWaitBusy();
}

static void rawSx126xReadReg(uint16_t addr, uint8_t *out, size_t n) {
    SPI.beginTransaction(loraSpi);
    digitalWrite(PIN_LORA_CS, LOW);
    SPI.transfer(0x1D);                 // ReadRegister
    SPI.transfer(addr >> 8);
    SPI.transfer(addr & 0xFF);
    SPI.transfer(0x00);                 // mandatory NOP before the data
    for (size_t i = 0; i < n; i++) out[i] = SPI.transfer(0x00);
    digitalWrite(PIN_LORA_CS, HIGH);
    SPI.endTransaction();
    rawWaitBusy();
}

static void rawSx126xWriteReg(uint16_t addr, const uint8_t *in, size_t n) {
    SPI.beginTransaction(loraSpi);
    digitalWrite(PIN_LORA_CS, LOW);
    SPI.transfer(0x0D);                 // WriteRegister
    SPI.transfer(addr >> 8);
    SPI.transfer(addr & 0xFF);
    for (size_t i = 0; i < n; i++) SPI.transfer(in[i]);
    digitalWrite(PIN_LORA_CS, HIGH);
    SPI.endTransaction();
    rawWaitBusy();
}

static uint16_t rawSx126xDeviceErrors() {
    uint8_t b[2] = { 0, 0 };
    SPI.beginTransaction(loraSpi);
    digitalWrite(PIN_LORA_CS, LOW);
    SPI.transfer(0x17);                 // GetDeviceErrors
    SPI.transfer(0x00);                 // status byte
    b[0] = SPI.transfer(0x00);
    b[1] = SPI.transfer(0x00);
    digitalWrite(PIN_LORA_CS, HIGH);
    SPI.endTransaction();
    rawWaitBusy();
    return ((uint16_t)b[0] << 8) | b[1];
}

// Every opcode used while probing has bit 7 clear. That matters: an
// SX127x decodes a first byte >= 0x80 as a register *write*, so a
// misdirected SX126x command could scribble on RegPaConfig or even key
// the transmitter (0xD1 = SetTxContinuousWave). Below 0x80 the worst an
// SX127x does is read a register back at us.
static bool sx126xPresent() {
    // LoRa sync word register, reset value 0x1424. An SX1276 decodes the
    // 0x1D opcode as a read of RegModemConfig1 and burst-reads onwards,
    // returning 0x0000 in these positions, so this cannot false-positive.
    uint8_t sync[2] = { 0, 0 };
    rawSx126xReadReg(0x0740, sync, 2);
    if (sync[0] == 0x14 && sync[1] == 0x24) return true;

    // Fallback for a chip left in a non-default state: write a pattern
    // and read it back. On an SX127x the 0x0D opcode has bit 7 clear, so
    // it is a read of RegFifoAddrPtr and changes nothing.
    const uint8_t pattern[2] = { 0xAB, 0xCD };
    rawSx126xWriteReg(0x0740, pattern, 2);
    uint8_t readback[2] = { 0, 0 };
    rawSx126xReadReg(0x0740, readback, 2);
    if (readback[0] == 0xAB && readback[1] == 0xCD) {
        const uint8_t restore[2] = { 0x14, 0x24 };
        rawSx126xWriteReg(0x0740, restore, 2);
        return true;
    }
    return false;
}

RadioChip TrackerRadio::probe() {
    rawBegin();
    rawReset();
    sx126xVersion[0] = '\0';

    // Test for the SX126x first. Its sync-word signature cannot be
    // produced by an SX1276, whereas the SX127x version byte sits in the
    // range an SX126x can legitimately return as a status byte for an
    // unrecognised opcode - so the SX126x test is the one to trust when
    // both could plausibly match.
    if (sx126xPresent()) {
        uint8_t v[8] = { 0 };
        rawSx126xReadReg(0x0320, v, 6);      // chip version string
        for (int i = 0; i < 6; i++) {
            sx126xVersion[i] = (v[i] >= 0x20 && v[i] < 0x7F) ? (char)v[i] : '?';
        }
        sx126xVersion[6] = '\0';
        return RADIO_CHIP_SX126X;
    }

    // RegVersion: 0x12 is the datasheet value, 0x13 appears in the
    // field, and RFM9x modules report 0x11. RadioLib accepts all three,
    // so the probe must too or a genuine V1 board is rejected. 0x22
    // (SX1272) is deliberately *not* accepted: this board never carries
    // one, and it is also a legal SX1262 status byte.
    uint8_t version = rawSx127xRead(0x42);
    if (version == 0x11 || version == 0x12 || version == 0x13) {
        return RADIO_CHIP_SX127X;
    }

    return RADIO_CHIP_NONE;
}

// ------------------------------------------------------------
// Oscillator source probe (SX126x)
//
// Modules come both ways: some clock the chip from a plain crystal,
// others from a TCXO that the SX1262 must power up through DIO3. Get it
// wrong in either direction and the oscillator never starts. Rather
// than hard-code it per board revision, ask the chip: bring it up, tune
// to the working frequency, lock the PLL, and read GetDeviceErrors.
// ------------------------------------------------------------
static bool oscillatorWorks(int tcxoVoltCode, float freqMHz) {
    rawReset();

    const uint8_t stdbyRc[1] = { 0x00 };
    rawSx126xCmd(0x80, stdbyRc, 1);                 // SetStandby(STDBY_RC)

    const uint8_t clearErr[2] = { 0x00, 0x00 };
    rawSx126xCmd(0x07, clearErr, 2);                // ClearDeviceErrors

    if (tcxoVoltCode >= 0) {
        // SetDIO3AsTcxoCtrl(voltage, 320 * 15.625us = 5 ms startup)
        const uint8_t p[4] = { (uint8_t)tcxoVoltCode, 0x00, 0x01, 0x40 };
        rawSx126xCmd(0x97, p, 4);
    }

    const uint8_t calibrateAll[1] = { 0x7F };
    rawSx126xCmd(0x89, calibrateAll, 1);            // Calibrate
    delay(20);

    uint32_t frf = (uint32_t)(freqMHz * 1048576.0f); // freq * 2^25 / 32 MHz
    const uint8_t f[4] = { (uint8_t)(frf >> 24), (uint8_t)(frf >> 16),
                           (uint8_t)(frf >> 8),  (uint8_t)frf };
    rawSx126xCmd(0x86, f, 4);                       // SetRfFrequency
    rawSx126xCmd(0xC1, nullptr, 0);                 // SetFs - locks the PLL
    delay(10);

    uint16_t err = rawSx126xDeviceErrors();
    rawSx126xCmd(0x80, stdbyRc, 1);

    // XOSC_START_ERR (bit 5) or PLL_LOCK_ERR (bit 6) means this clock
    // configuration is not the one the module is built for.
    return (err & 0x0060) == 0;
}

// Sets *ok false when no clock configuration produced a running
// oscillator and a locked PLL, which is worth saying out loud rather
// than silently falling back to the crystal setting.
static float probeTcxoVoltage(float freqMHz, bool *ok) {
    *ok = true;
    if (oscillatorWorks(-1, freqMHz))   return 0.0f;   // plain crystal
    if (oscillatorWorks(0x02, freqMHz)) return 1.8f;
    if (oscillatorWorks(0x07, freqMHz)) return 3.3f;
    if (oscillatorWorks(0x06, freqMHz)) return 3.0f;
    if (oscillatorWorks(0x05, freqMHz)) return 2.7f;
    *ok = false;
    return 0.0f;
}

// ============================================================
// Bring-up
// ============================================================

static RadioChip resolveChip(const TrackerConfig &cfg) {
    if (strcasecmp(cfg.radioChip, "sx1276") == 0 ||
        strcasecmp(cfg.radioChip, "sx127x") == 0 ||
        strcasecmp(cfg.radioChip, "rfm95")  == 0) {
        return RADIO_CHIP_SX127X;
    }
    if (strcasecmp(cfg.radioChip, "sx1262") == 0 ||
        strcasecmp(cfg.radioChip, "sx126x") == 0 ||
        strcasecmp(cfg.radioChip, "sx1268") == 0) {
        return RADIO_CHIP_SX126X;
    }
    return TrackerRadio::probe();
}

// Does the fitted module carry its own amplifier?
//
// The radio itself is the only reliable indicator. Every board in this
// family that carries an SX126x carries it inside a 1262MiniF27, and
// every SX127x board is a bare RFM95. There is no strap worth reading:
// GP15 was assumed to mark the revision and turned out to read low on
// V1 hardware too, so it distinguished nothing.
//
// `radioModule=bare` remains the escape hatch if a plain SX1262 module
// is ever fitted, since driving one as though it fed an amplifier would
// under-power it badly.
static bool resolveModulePa(const TrackerConfig &cfg, RadioChip detected) {
    if (strcasecmp(cfg.radioModule, "minif27") == 0) return detected == RADIO_CHIP_SX126X;
    if (strcasecmp(cfg.radioModule, "bare") == 0)    return false;
    return detected == RADIO_CHIP_SX126X;
}

// Low data rate optimization is mandatory once a symbol lasts 16 ms or
// more, which SF12 at 125 kHz does. Both families must agree or the
// packet will not decode.
static bool ldroNeeded() {
    float symbolMs = (float)(1UL << LORA_SF) / LORA_BW_KHZ;
    return symbolMs >= 16.0f;
}

bool TrackerRadio::begin(const TrackerConfig &cfg, char *err, size_t errLen) {
    rawBegin();

    activeChip = resolveChip(cfg);
    appliedPwr = 0;
    tcxoVolts  = 0.0f;
    phy        = nullptr;
    modulePa   = resolveModulePa(cfg, activeChip);

    // resolveChip() may have short-circuited the probe because the
    // family was pinned in config, in which case the version string was
    // never read and an SX1268 would be handed to the SX1262 class,
    // which rejects it with ERR_CHIP_NOT_FOUND. Read it now.
    if (activeChip == RADIO_CHIP_SX126X && sx126xVersion[0] == '\0') {
        rawReset();
        uint8_t v[8] = { 0 };
        rawSx126xReadReg(0x0320, v, 6);
        for (int i = 0; i < 6; i++) {
            sx126xVersion[i] = (v[i] >= 0x20 && v[i] < 0x7F) ? (char)v[i] : '?';
        }
        sx126xVersion[6] = '\0';
    }

    if (activeChip == RADIO_CHIP_NONE) {
        snprintf(err, errLen, "no LoRa radio found on SPI (CS=GP%d RST=GP%d)",
                 PIN_LORA_CS, PIN_LORA_RST);
        return false;
    }

    int16_t state = RADIOLIB_ERR_NONE;
    int8_t  power = (int8_t)cfg.power;
    int8_t  clipped = power;

    if (activeChip == RADIO_CHIP_SX127X) {
        // RadioLib's SX1276 PA_BOOST path can only produce 2..17 plus
        // the discrete high-power point at 20; 18 and 19 are rejected
        // outright, and begin() would fail rather than clamp. Map to the
        // nearest producible value that does not exceed the request, so
        // 23 lands on 20 - the same PaDac 0x87 / PA_BOOST state the V1
        // firmware programmed - while 18 and 19 round down to 17 rather
        // than up to 20.
        clipped = (power >= 20) ? 20 : (int8_t)constrain(power, 2, 17);
        state = radioSx127x.begin(cfg.loraFrequency, LORA_BW_KHZ, LORA_SF,
                                  LORA_CR, LORA_SYNC_WORD, clipped,
                                  LORA_PREAMBLE);
        if (state != RADIOLIB_ERR_NONE) {
            snprintf(err, errLen, "SX1276 begin failed (%d)", state);
            return false;
        }
        radioSx127x.setCRC(true);
        radioSx127x.explicitHeader();
        radioSx127x.forceLDRO(ldroNeeded());
        radioSx127x.setCurrentLimit(LORA_OCP_SX127X_MA);

        phy        = &radioSx127x;
        txDoneMask = RADIOLIB_SX127X_CLEAR_IRQ_FLAG_TX_DONE;

    } else {
        // Decide crystal vs TCXO before RadioLib takes over: begin()
        // reports success either way, but the oscillator silently fails
        // to start if the choice is wrong.
        if (strcasecmp(cfg.loraTcxo, "auto") == 0) {
            bool oscOk = true;
            tcxoVolts = probeTcxoVoltage(cfg.loraFrequency, &oscOk);
            if (!oscOk) {
                snprintf(err, errLen,
                         "SX126x oscillator would not start on crystal or any TCXO voltage");
                return false;
            }
        } else {
            tcxoVolts = atof(cfg.loraTcxo);          // "0" = plain crystal
        }

        // SX1261 and SX1262 both report "SX1261"; only an SX1268 says
        // otherwise, and RadioLib's SX1262 class refuses to start on one.
        bool is1268 = (strncmp(sx126xVersion, "SX1268", 6) == 0);

        // On a module with its own amplifier the chip output is the
        // drive into that amplifier rather than the radiated power, so
        // it gets its own config key and is not confused with `power`.
        clipped = modulePa
                    ? (int8_t)constrain(cfg.paDrive, MINIF27_MIN_DRIVE, MINIF27_MAX_DRIVE)
                    : (int8_t)constrain(power, -9, 22);

        // DC-DC halves the die's supply current but needs an inductor
        // on the module's DCC_SW pin. Where there is none, the die's PA
        // is starved and the module's amplifier faithfully amplifies a
        // weak drive - so this is configurable rather than assumed.
        useLdoReg = (strcasecmp(cfg.loraRegulator, "ldo") == 0);

        if (is1268) {
            state = radioSx1268.begin(cfg.loraFrequency, LORA_BW_KHZ, LORA_SF,
                                      LORA_CR, LORA_SYNC_WORD, clipped,
                                      LORA_PREAMBLE, tcxoVolts, useLdoReg);
            sx126 = &radioSx1268;
            phy   = &radioSx1268;
        } else {
            state = radioSx1262.begin(cfg.loraFrequency, LORA_BW_KHZ, LORA_SF,
                                      LORA_CR, LORA_SYNC_WORD, clipped,
                                      LORA_PREAMBLE, tcxoVolts, useLdoReg);
            sx126 = &radioSx1262;
            phy   = &radioSx1262;
        }
        if (state != RADIOLIB_ERR_NONE) {
            snprintf(err, errLen, "%s begin failed (%d)",
                     is1268 ? "SX1268" : "SX1262", state);
            phy = nullptr;
            return false;
        }

        // A TCXO is only powered up once SetDIO3AsTcxoCtrl has run, so
        // everything calibrated before that used a dead clock. RadioLib
        // does not recalibrate afterwards - the datasheet requires it.
        if (tcxoVolts > 0.0f) {
            sx126->calibrate(RADIOLIB_SX126X_CALIBRATE_ALL);
            delay(20);
            phy->setFrequency(cfg.loraFrequency);
        }

        sx126->setCRC(2);
        sx126->forceLDRO(ldroNeeded());
        // begin() already enables DIO2 as the RF switch control, which
        // is what module-integrated switches expect and is harmless on
        // modules without one.
        //
        // Last, because setOutputPower() saves and restores whatever OCP
        // was in force when it ran.
        ocpMa = (float)cfg.loraOcp;
        sx126->setCurrentLimit(ocpMa);

        txDoneMask = RADIOLIB_SX126X_IRQ_TX_DONE;
    }

    appliedPwr = clipped;
    phy->standby();
    return true;
}

// ============================================================
// Transmit
// ============================================================

bool TrackerRadio::send(const uint8_t *data, size_t len) {
    if (!phy) return false;

    watchdog_update();

    // Note on the SX126x: RadioLib waits for BUSY to fall after SetTx
    // with no timeout of its own, so a module whose PA supply is absent
    // can spin there. Checking BUSY before the call would prove nothing,
    // because SetTx is what asserts it. The precondition is removed
    // instead - loraSendText() always powers the amplifier first - and
    // the 5 s watchdog is the backstop if it ever happens anyway.

    int16_t state = phy->startTransmit(data, len);
    if (state != RADIOLIB_ERR_NONE) {
        phy->finishTransmit();
        return false;
    }

    // An SF12 packet of this length is on the air for several seconds,
    // far longer than the 5 s watchdog, so the poll loop must keep
    // feeding it. Allow twice the nominal time on air before giving up.
    uint32_t budgetMs = timeOnAirMs(len) * 2 + 1000;
    uint32_t start    = millis();
    bool     done     = false;

    while ((millis() - start) < budgetMs) {
        watchdog_update();
        if (phy->getIrqFlags() & txDoneMask) {
            done = true;
            break;
        }
        delay(2);
    }

    phy->finishTransmit();
    watchdog_update();
    return done;
}

// Change the drive between packets.
//
// The clamping mirrors begin() exactly, so a value that was legal at
// boot stays legal here. On the SX126x setOutputPower() saves and
// restores whatever OCP was in force when it ran, which silently undoes
// the configured current limit - so it is re-applied every time.
bool TrackerRadio::setDrive(int8_t dbm) {
    if (!phy) return false;

    int8_t clipped;
    if (activeChip == RADIO_CHIP_SX127X) {
        clipped = (dbm >= 20) ? 20 : (int8_t)constrain(dbm, 2, 17);
        if (clipped == appliedPwr) return true;
        if (radioSx127x.setOutputPower(clipped) != RADIOLIB_ERR_NONE) return false;
        radioSx127x.setCurrentLimit(LORA_OCP_SX127X_MA);
    } else {
        clipped = modulePa
                    ? (int8_t)constrain(dbm, MINIF27_MIN_DRIVE, MINIF27_MAX_DRIVE)
                    : (int8_t)constrain(dbm, -9, 22);
        if (clipped == appliedPwr) return true;
        if (sx126->setOutputPower(clipped) != RADIOLIB_ERR_NONE) return false;
        sx126->setCurrentLimit(ocpMa);
    }

    appliedPwr = clipped;
    return true;
}

// ============================================================
// Accessors
// ============================================================

RadioChip TrackerRadio::chip() { return activeChip; }

const char *TrackerRadio::chipName() {
    switch (activeChip) {
        case RADIO_CHIP_SX127X:
            return "SX1276/RFM95";
        case RADIO_CHIP_SX126X:
            // "SX1261" is what both the SX1261 and the SX1262 report.
            return (strncmp(sx126xVersion, "SX1268", 6) == 0) ? "SX1268"
                                                              : "SX1262";
        default:
            return "none";
    }
}

int8_t TrackerRadio::appliedPower() { return appliedPwr; }

bool TrackerRadio::hasModulePa() { return modulePa; }

float TrackerRadio::tcxoVoltage() { return tcxoVolts; }

bool TrackerRadio::usingLdoRegulator() { return useLdoReg; }

uint32_t TrackerRadio::timeOnAirMs(size_t len) {
    if (!phy) return 0;
    return (uint32_t)(phy->getTimeOnAir(len) / 1000);
}

void TrackerRadio::describe(char *buf, size_t len) {
    // A typical APRS position frame, for a representative time on air.
    const size_t sampleLen = 96;

    if (activeChip == RADIO_CHIP_SX126X) {
        uint8_t sync[2] = { 0, 0 };
        uint8_t ocp = 0;
        rawSx126xReadReg(0x0740, sync, 2);
        rawSx126xReadReg(0x08E7, &ocp, 1);
        uint16_t errors = rawSx126xDeviceErrors();
        snprintf(buf, len,
                 "SX126x ver=%s sync=0x%02X%02X ocp=0x%02X(%umA) errors=0x%04X toa(%uB)=%lums",
                 sx126xVersion, sync[0], sync[1], ocp, (unsigned)(ocp * 5 / 2),
                 errors, (unsigned)sampleLen,
                 (unsigned long)timeOnAirMs(sampleLen));

    } else if (activeChip == RADIO_CHIP_SX127X) {
        snprintf(buf, len,
                 "SX127x sync=0x%02X cfg1=0x%02X cfg2=0x%02X cfg3=0x%02X "
                 "pa=0x%02X padac=0x%02X ocp=0x%02X toa(%uB)=%lums",
                 rawSx127xRead(0x39), rawSx127xRead(0x1D), rawSx127xRead(0x1E),
                 rawSx127xRead(0x26), rawSx127xRead(0x09), rawSx127xRead(0x4D),
                 rawSx127xRead(0x0B),
                 (unsigned)sampleLen, (unsigned long)timeOnAirMs(sampleLen));

    } else {
        snprintf(buf, len, "no radio");
    }
}
