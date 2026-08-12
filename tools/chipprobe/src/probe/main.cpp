// ============================================================
// RF.Guru LoRa Radio Chip Probe
//
// Standalone diagnostic firmware for the LoRa433APRSTracker PCB.
//
//   1. Identifies the fitted radio: SX127x (RFM9x) or SX126x (SX1262).
//   2. Empirically discovers which RP2040 GPIO carries SX126x BUSY
//      and DIO1/DIO2/DIO3 (or SX127x DIO0) by watching every free
//      GPIO while the radio is made to assert those lines.
//   3. Reports whether the module needs a TCXO on DIO3, and at what
//      voltage, by looking at XOSC_START_ERR from GetDeviceErrors.
//   4. Checks the PLL locks at 433.775 MHz.
//   5. Verifies the reset line actually resets the chip.
//
// SAFETY: read/receive only. Never issues SetTx, never enables the
// external PA (GP2 is held low the whole time). No RF is emitted.
// ============================================================

#include <Arduino.h>
#include <SPI.h>
#include <hardware/gpio.h>

// ------------------------------------------------------------
// Known board wiring (from include/pins.h of the V1 design)
// ------------------------------------------------------------
#define PIN_LED_PWR   9
#define PIN_LED_GPS  10
#define PIN_LED_LORA 11
#define PIN_PA        2
#define PIN_I2C_PWR   3
#define PIN_GPS_TX    4
#define PIN_GPS_RX    5
#define PIN_GPS_RST  12
#define PIN_I2C_SDA   6
#define PIN_I2C_SCL   7
#define PIN_SPI_MISO 16
#define PIN_SPI_CLK  18
#define PIN_SPI_MOSI 19
#define PIN_LORA_RST 20
#define PIN_LORA_CS  21

// Discovered by the v1 probe run.
#define PIN_LORA_BUSY 22

// Every GPIO we may reconfigure as an input and watch. Excludes the
// SPI bus, CS/RST, and GP2 (PA enable - must stay driven low so the
// amplifier can never be keyed).
static const uint8_t SCAN_PINS[] = {
    0, 1, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 17,
    22, 23, 24, 25, 26, 27, 28, 29
};
static const size_t N_SCAN = sizeof(SCAN_PINS) / sizeof(SCAN_PINS[0]);
static uint32_t scanMask = 0;

static SPISettings spiCfg(2000000, MSBFIRST, SPI_MODE0);
static uint8_t csPin = PIN_LORA_CS;
static bool busyKnown = false;      // set once BUSY is confirmed on GP22

// ------------------------------------------------------------
// Output helpers
// ------------------------------------------------------------
static void hdr(const char *s) {
    Serial.printf("\r\n\x1b[1;36m=== %s ===\x1b[0m\r\n", s);
}
static void ok(const char *fmt, ...) {
    char b[256];
    va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof(b), fmt, ap); va_end(ap);
    Serial.printf("\x1b[1;32m%s\x1b[0m\r\n", b);
}
static void warn(const char *fmt, ...) {
    char b[256];
    va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof(b), fmt, ap); va_end(ap);
    Serial.printf("\x1b[38;5;220m%s\x1b[0m\r\n", b);
}
static void info(const char *fmt, ...) {
    char b[256];
    va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof(b), fmt, ap); va_end(ap);
    Serial.printf("%s\r\n", b);
}

// ------------------------------------------------------------
// Pin state management
// ------------------------------------------------------------
// Restores every pin the board expects us to drive. Called after any
// scan that temporarily floated them.
static void restoreDrivenPins() {
    pinMode(PIN_PA, OUTPUT);        digitalWrite(PIN_PA, LOW);
    pinMode(PIN_I2C_PWR, OUTPUT);   digitalWrite(PIN_I2C_PWR, LOW);
    pinMode(PIN_LED_PWR, OUTPUT);   digitalWrite(PIN_LED_PWR, HIGH);
    pinMode(PIN_LED_GPS, OUTPUT);   digitalWrite(PIN_LED_GPS, LOW);
    pinMode(PIN_LED_LORA, OUTPUT);  digitalWrite(PIN_LED_LORA, LOW);
    pinMode(PIN_GPS_RST, OUTPUT);   digitalWrite(PIN_GPS_RST, HIGH);
}

static void armScanPins(int pullMode) {
    for (size_t i = 0; i < N_SCAN; i++) pinMode(SCAN_PINS[i], pullMode);
    delay(5);
}

static void printMask(const char *label, uint32_t m) {
    Serial.printf("%-30s", label);
    bool any = false;
    for (size_t i = 0; i < N_SCAN; i++) {
        if (m & (1u << SCAN_PINS[i])) { Serial.printf(" GP%d", SCAN_PINS[i]); any = true; }
    }
    if (!any) Serial.print(" (none)");
    Serial.print("\r\n");
}

static uint32_t sampleHighs(uint32_t us) {
    uint32_t acc = 0;
    uint32_t t0 = micros();
    while ((uint32_t)(micros() - t0) < us) acc |= gpio_get_all();
    return acc;
}

// ------------------------------------------------------------
// Low level SPI
// ------------------------------------------------------------
static inline void csLow()  { digitalWrite(csPin, LOW);  }
static inline void csHigh() { digitalWrite(csPin, HIGH); }

// Once BUSY is located we can wait on it properly instead of guessing.
static void waitBusy(uint32_t timeoutUs = 20000) {
    if (!busyKnown) { delayMicroseconds(1000); return; }
    uint32_t t0 = micros();
    while (gpio_get(PIN_LORA_BUSY)) {
        if ((uint32_t)(micros() - t0) > timeoutUs) return;
    }
}

static void radioReset() {
    pinMode(PIN_LORA_RST, OUTPUT);
    digitalWrite(PIN_LORA_RST, LOW);
    delay(5);
    digitalWrite(PIN_LORA_RST, HIGH);
    delay(30);
}

static uint8_t sx127xRead(uint8_t addr) {
    SPI.beginTransaction(spiCfg);
    csLow();
    SPI.transfer(addr & 0x7F);
    uint8_t v = SPI.transfer(0x00);
    csHigh();
    SPI.endTransaction();
    return v;
}

static void sx127xWrite(uint8_t addr, uint8_t val) {
    SPI.beginTransaction(spiCfg);
    csLow();
    SPI.transfer(addr | 0x80);
    SPI.transfer(val);
    csHigh();
    SPI.endTransaction();
}

static void sx126xCmd(uint8_t opcode, const uint8_t *data, size_t n,
                      uint32_t settleUs = 1000) {
    SPI.beginTransaction(spiCfg);
    csLow();
    SPI.transfer(opcode);
    for (size_t i = 0; i < n; i++) SPI.transfer(data[i]);
    csHigh();
    SPI.endTransaction();
    if (busyKnown) waitBusy(); else delayMicroseconds(settleUs);
}

static void sx126xCmdRead(uint8_t opcode, uint8_t *out, size_t n) {
    SPI.beginTransaction(spiCfg);
    csLow();
    SPI.transfer(opcode);
    SPI.transfer(0x00);
    for (size_t i = 0; i < n; i++) out[i] = SPI.transfer(0x00);
    csHigh();
    SPI.endTransaction();
    waitBusy();
}

static uint8_t sx126xGetStatus() {
    SPI.beginTransaction(spiCfg);
    csLow();
    SPI.transfer(0xC0);
    uint8_t st = SPI.transfer(0x00);
    csHigh();
    SPI.endTransaction();
    waitBusy();
    return st;
}

static void sx126xReadReg(uint16_t addr, uint8_t *out, size_t n) {
    SPI.beginTransaction(spiCfg);
    csLow();
    SPI.transfer(0x1D);
    SPI.transfer(addr >> 8);
    SPI.transfer(addr & 0xFF);
    SPI.transfer(0x00);
    for (size_t i = 0; i < n; i++) out[i] = SPI.transfer(0x00);
    csHigh();
    SPI.endTransaction();
    waitBusy();
}

static void sx126xWriteReg(uint16_t addr, const uint8_t *in, size_t n) {
    SPI.beginTransaction(spiCfg);
    csLow();
    SPI.transfer(0x0D);
    SPI.transfer(addr >> 8);
    SPI.transfer(addr & 0xFF);
    for (size_t i = 0; i < n; i++) SPI.transfer(in[i]);
    csHigh();
    SPI.endTransaction();
    waitBusy();
}

static uint16_t sx126xDeviceErrors() {
    uint8_t b[2] = { 0, 0 };
    sx126xCmdRead(0x17, b, 2);
    return ((uint16_t)b[0] << 8) | b[1];
}

static void sx126xClearDeviceErrors() {
    const uint8_t p[2] = { 0x00, 0x00 };
    sx126xCmd(0x07, p, 2);
}

static void sx126xStandbyRC() {
    const uint8_t p[1] = { 0x00 };
    sx126xCmd(0x80, p, 1, 2000);
}

// ------------------------------------------------------------
// Detection
// ------------------------------------------------------------
enum ChipKind { CHIP_NONE, CHIP_SX127X, CHIP_SX126X };

static ChipKind detectChip(uint8_t &sx127xVersion, uint8_t sync[2], uint8_t &status) {
    radioReset();
    sx127xVersion = sx127xRead(0x42);
    status = sx126xGetStatus();
    sx126xReadReg(0x0740, sync, 2);

    if (sx127xVersion == 0x12 || sx127xVersion == 0x22) return CHIP_SX127X;
    if (sync[0] == 0x14 && sync[1] == 0x24) return CHIP_SX126X;

    const uint8_t pat[2] = { 0xAB, 0xCD };
    sx126xWriteReg(0x0740, pat, 2);
    uint8_t rb[2] = { 0, 0 };
    sx126xReadReg(0x0740, rb, 2);
    if (rb[0] == 0xAB && rb[1] == 0xCD) {
        const uint8_t restore[2] = { 0x14, 0x24 };
        sx126xWriteReg(0x0740, restore, 2);
        return CHIP_SX126X;
    }
    return CHIP_NONE;
}

// Prove GP20 really resets the radio: dirty a register, pulse reset,
// confirm the POR value is back.
static void verifyReset() {
    hdr("Reset line verification (GP20)");
    const uint8_t pat[2] = { 0xAB, 0xCD };
    sx126xWriteReg(0x0740, pat, 2);
    uint8_t rb[2];
    sx126xReadReg(0x0740, rb, 2);
    info("  after writing 0xABCD : 0x%02X%02X", rb[0], rb[1]);
    radioReset();
    sx126xReadReg(0x0740, rb, 2);
    info("  after pulsing GP20   : 0x%02X%02X", rb[0], rb[1]);
    if (rb[0] == 0x14 && rb[1] == 0x24) ok("  => GP20 is a working NRESET");
    else warn("  => GP20 did NOT reset the chip");
}

// ------------------------------------------------------------
// BUSY discovery
// ------------------------------------------------------------
static uint32_t findBusyPin() {
    hdr("BUSY pin discovery");

    busyKnown = false;
    radioReset();
    sx126xStandbyRC();

    armScanPins(INPUT_PULLDOWN);
    uint32_t control = sampleHighs(8000);

    SPI.beginTransaction(spiCfg);
    csLow(); SPI.transfer(0x89); SPI.transfer(0x7F); csHigh();
    SPI.endTransaction();
    uint32_t during = sampleHighs(8000);
    delay(50);
    uint32_t after = gpio_get_all();

    uint32_t control2 = sampleHighs(4000);
    SPI.beginTransaction(spiCfg);
    csLow(); SPI.transfer(0x80); SPI.transfer(0x01); csHigh();   // STDBY_XOSC
    SPI.endTransaction();
    uint32_t during2 = sampleHighs(4000);
    delay(50);
    uint32_t after2 = gpio_get_all();

    uint32_t cand1 = during  & ~control  & ~after  & scanMask;
    uint32_t cand2 = during2 & ~control2 & ~after2 & scanMask;
    uint32_t both  = cand1 & cand2;

    printMask("high during Calibrate:", cand1);
    printMask("high during STDBY_XOSC:", cand2);
    if (both) printMask("\x1b[1;32m=> BUSY is on:\x1b[0m", both);
    else      warn("=> BUSY not found on any scanned GPIO");

    restoreDrivenPins();
    if (both & (1u << PIN_LORA_BUSY)) busyKnown = true;
    return both;
}

// ------------------------------------------------------------
// Per-DIO discovery. Routes every LoRa IRQ to exactly one DIO, then
// forces an RX timeout (receive only - no RF emitted) and watches
// which GPIO latches high and then drops when the IRQ is cleared.
// ------------------------------------------------------------
static uint32_t irqRouteTest(int dioIdx, const char *name) {
    radioReset();
    busyKnown = false;                 // BUSY state unknown right after reset
    sx126xStandbyRC();
    busyKnown = true;

    const uint8_t pktLora[1] = { 0x01 };
    sx126xCmd(0x8A, pktLora, 1);       // SetPacketType(LORA)

    uint8_t irq[8] = { 0xFF, 0xFF, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
    irq[2 + dioIdx * 2] = 0xFF;
    irq[3 + dioIdx * 2] = 0xFF;
    sx126xCmd(0x08, irq, 8);           // SetDioIrqParams

    const uint8_t clr[2] = { 0xFF, 0xFF };
    sx126xCmd(0x02, clr, 2);

    armScanPins(INPUT_PULLDOWN);
    uint32_t before = sampleHighs(4000);

    const uint8_t rx[3] = { 0x00, 0x0F, 0xA0 };   // 62.5 ms timeout
    SPI.beginTransaction(spiCfg);
    csLow(); SPI.transfer(0x82);
    SPI.transfer(rx[0]); SPI.transfer(rx[1]); SPI.transfer(rx[2]);
    csHigh();
    SPI.endTransaction();

    uint32_t during = sampleHighs(150000);
    delay(20);
    uint32_t held = gpio_get_all();

    uint8_t st[2] = { 0, 0 };
    sx126xCmdRead(0x12, st, 2);
    uint16_t irqBits = ((uint16_t)st[0] << 8) | st[1];

    sx126xCmd(0x02, clr, 2);
    delay(5);
    uint32_t cleared = gpio_get_all();

    // Ignore BUSY: it pulses on every command regardless of routing.
    uint32_t ignore = (1u << PIN_LORA_BUSY);
    uint32_t latched = (held & ~before & scanMask & ~ignore) & ~cleared;
    uint32_t pulsed  = (during & ~before & scanMask & ~ignore);

    info("  %s: IrqStatus=0x%04X %s", name, irqBits,
         (irqBits & 0x0200) ? "(Timeout set)" : "(NO timeout!)");
    printMask("    pulsed high:", pulsed);
    printMask("    latched then cleared:", latched);
    if (latched) printMask("\x1b[1;32m    => this DIO is on:\x1b[0m", latched);

    restoreDrivenPins();
    return latched;
}

static void findDioPins() {
    hdr("DIO1 / DIO2 / DIO3 discovery (RX-timeout IRQ, no TX)");
    uint32_t d1 = irqRouteTest(0, "DIO1");
    uint32_t d2 = irqRouteTest(1, "DIO2");
    uint32_t d3 = irqRouteTest(2, "DIO3");
    Serial.print("\r\n");
    if (!d1) warn("  DIO1 is NOT wired to the MCU -> firmware must poll IrqStatus");
    if (!d2) info("  DIO2 not wired to the MCU (normal: it drives the RF switch)");
    if (!d3) info("  DIO3 not wired to the MCU (normal: it powers a TCXO)");
}

// Identify DIO3 a second way: SetDIO3AsTcxoCtrl drives DIO3 to the
// selected voltage. At 3.3 V that is a clean logic high on the RP2040.
static void findDio3ViaTcxo() {
    hdr("DIO3 identification via SetDIO3AsTcxoCtrl @ 3.3V");
    radioReset();
    busyKnown = false;
    sx126xStandbyRC();

    armScanPins(INPUT_PULLDOWN);
    uint32_t before = sampleHighs(4000);

    const uint8_t p[4] = { 0x07, 0x00, 0x01, 0x40 };   // 3.3 V, 5 ms
    SPI.beginTransaction(spiCfg);
    csLow(); SPI.transfer(0x97);
    for (int i = 0; i < 4; i++) SPI.transfer(p[i]);
    csHigh();
    SPI.endTransaction();
    delay(20);

    uint32_t after = gpio_get_all();
    uint32_t cand = after & ~before & scanMask & ~(1u << PIN_LORA_BUSY);
    printMask("high after TCXO enable:", cand);
    if (cand) printMask("\x1b[1;32m=> DIO3 is on:\x1b[0m", cand);
    else      info("=> DIO3 is not routed to the MCU");

    restoreDrivenPins();
    radioReset();
    busyKnown = true;
}

// ------------------------------------------------------------
// Oscillator test
// ------------------------------------------------------------
static void decodeErrors(uint16_t e) {
    if (!e) { ok("    device errors: none (0x0000)"); return; }
    Serial.printf("    device errors: 0x%04X ->", e);
    if (e & 0x0001) Serial.print(" RC64K_CALIB");
    if (e & 0x0002) Serial.print(" RC13M_CALIB");
    if (e & 0x0004) Serial.print(" PLL_CALIB");
    if (e & 0x0008) Serial.print(" ADC_CALIB");
    if (e & 0x0010) Serial.print(" IMG_CALIB");
    if (e & 0x0020) Serial.print(" \x1b[1;31mXOSC_START\x1b[0m");
    if (e & 0x0040) Serial.print(" \x1b[1;31mPLL_LOCK\x1b[0m");
    if (e & 0x0100) Serial.print(" PA_RAMP");
    Serial.print("\r\n");
}

static uint16_t bringUp(int tcxoVoltCode, const char *label) {
    radioReset();
    busyKnown = false;
    sx126xStandbyRC();
    busyKnown = true;
    sx126xClearDeviceErrors();

    if (tcxoVoltCode >= 0) {
        const uint8_t p[4] = { (uint8_t)tcxoVoltCode, 0x00, 0x01, 0x40 };
        sx126xCmd(0x97, p, 4, 10000);
    }

    const uint8_t calAll[1] = { 0x7F };
    sx126xCmd(0x89, calAll, 1, 20000);
    delay(20);

    const uint32_t frf = 454846054UL;                  // 433.775 MHz
    const uint8_t f[4] = { (uint8_t)(frf >> 24), (uint8_t)(frf >> 16),
                           (uint8_t)(frf >> 8),  (uint8_t)frf };
    sx126xCmd(0x86, f, 4, 2000);
    sx126xCmd(0xC1, nullptr, 0, 5000);                 // SetFs - lock PLL
    delay(10);

    uint16_t e = sx126xDeviceErrors();
    uint8_t st = sx126xGetStatus();
    info("  %-26s status=0x%02X chipMode=%d cmdStatus=%d", label, st,
         (st >> 4) & 0x07, (st >> 1) & 0x07);
    decodeErrors(e);
    sx126xStandbyRC();
    return e;
}

static void testTcxo() {
    hdr("Oscillator source test");
    uint16_t eXtal = bringUp(-1, "plain XTAL:");
    if (!(eXtal & 0x0060))
        ok("=> Crystal module: XOSC starts and PLL locks at 433.775 MHz");
    else
        warn("=> XTAL path failed - module likely needs a TCXO");

    static const uint8_t volts[] = { 0x02, 0x07 };
    static const char *names[]   = { "TCXO on DIO3 @1.8V:", "TCXO on DIO3 @3.3V:" };
    for (size_t i = 0; i < sizeof(volts); i++) {
        uint16_t e = bringUp(volts[i], names[i]);
        if (!(e & 0x0060)) ok("    -> also works with this TCXO setting");
    }
    bringUp(-1, "back to XTAL:");
}

// ------------------------------------------------------------
// Register dump
// ------------------------------------------------------------
static void dumpSx126x() {
    hdr("SX126x register dump (post-reset defaults)");
    uint8_t r[2];
    sx126xReadReg(0x0740, r, 2); info("  0x0740 LoRaSyncWord   = 0x%02X%02X  (POR 0x1424)", r[0], r[1]);
    sx126xReadReg(0x08E7, r, 1); info("  0x08E7 OCP            = 0x%02X    (0x18=60mA, 0x38=140mA)", r[0]);
    sx126xReadReg(0x0889, r, 1); info("  0x0889 TxModulation   = 0x%02X", r[0]);
    sx126xReadReg(0x08D8, r, 1); info("  0x08D8 TxClampConfig  = 0x%02X", r[0]);
    sx126xReadReg(0x0911, r, 1); info("  0x0911 XTAtrim        = 0x%02X", r[0]);
    sx126xReadReg(0x0912, r, 1); info("  0x0912 XTBtrim        = 0x%02X", r[0]);
    sx126xReadReg(0x0736, r, 1); info("  0x0736 IQPolarity     = 0x%02X", r[0]);
    sx126xReadReg(0x0702, r, 2); info("  0x0702 XtalCap?       = 0x%02X%02X", r[0], r[1]);
}

// ------------------------------------------------------------
// GPIO connectivity map
// ------------------------------------------------------------
static void gpioMap() {
    hdr("GPIO idle map (pull-up vs pull-down)");
    armScanPins(INPUT_PULLUP);
    uint32_t up = gpio_get_all();
    armScanPins(INPUT_PULLDOWN);
    uint32_t dn = gpio_get_all();
    restoreDrivenPins();
    for (size_t i = 0; i < N_SCAN; i++) {
        uint8_t p = SCAN_PINS[i];
        bool u = up & (1u << p), d = dn & (1u << p);
        const char *verdict = (u && d)   ? "driven HIGH externally"
                            : (!u && !d) ? "driven/pulled LOW externally"
                                         : "floating (not connected)";
        const char *note = "";
        if (p == PIN_LORA_BUSY) note = "  <- radio BUSY";
        else if (p == 26) note = "  <- ADC, no-PA voltage sense";
        else if (p == 27) note = "  <- ADC, PA voltage sense (13.8V divider)";
        else if (p == PIN_GPS_TX || p == PIN_GPS_RX) note = "  <- GPS UART";
        else if (p == PIN_I2C_SDA || p == PIN_I2C_SCL) note = "  <- I2C bus";
        else if (p == PIN_I2C_PWR) note = "  <- I2C power switch";
        else if (p == PIN_GPS_RST) note = "  <- GPS reset";
        else if (p == PIN_LED_PWR || p == PIN_LED_GPS || p == PIN_LED_LORA) note = "  <- LED";
        info("  GP%-2d  pullup=%d pulldown=%d  %-28s%s", p, u, d, verdict, note);
    }
}

// ------------------------------------------------------------
// SX127x path
// ------------------------------------------------------------
static void dumpSx127x() {
    hdr("SX127x register dump");
    static const uint8_t regs[] = { 0x01, 0x06, 0x07, 0x08, 0x09, 0x0A,
                                    0x1D, 0x1E, 0x26, 0x39, 0x42, 0x4D };
    static const char *names[]  = { "RegOpMode", "RegFrfMsb", "RegFrfMid",
                                    "RegFrfLsb", "RegPaConfig", "RegPaRamp",
                                    "RegModemConfig1", "RegModemConfig2",
                                    "RegModemConfig3", "RegSyncWord",
                                    "RegVersion", "RegPaDac" };
    for (size_t i = 0; i < sizeof(regs); i++)
        info("  0x%02X %-18s = 0x%02X", regs[i], names[i], sx127xRead(regs[i]));

    hdr("DIO0 discovery (SX127x CAD, no TX)");
    sx127xWrite(0x01, 0x00); delay(10);
    sx127xWrite(0x01, 0x80); delay(10);
    sx127xWrite(0x40, 0x80);
    sx127xWrite(0x12, 0xFF);
    sx127xWrite(0x01, 0x81);

    armScanPins(INPUT_PULLDOWN);
    uint32_t before = sampleHighs(4000);
    sx127xWrite(0x01, 0x87);
    uint32_t during = sampleHighs(200000);
    delay(10);
    uint32_t held = gpio_get_all();
    uint8_t flags = sx127xRead(0x12);
    sx127xWrite(0x12, 0xFF);
    delay(5);
    uint32_t cleared = gpio_get_all();
    sx127xWrite(0x01, 0x81);

    info("RegIrqFlags after CAD: 0x%02X %s", flags,
         (flags & 0x04) ? "(CadDone set)" : "(no CadDone)");
    printMask("high during CAD:", during & ~before & scanMask);
    uint32_t dropped = (held & ~before & scanMask) & ~cleared;
    if (dropped) printMask("\x1b[1;32m=> DIO0 is on:\x1b[0m", dropped);
    else         warn("=> DIO0 not wired to the MCU (matches the V1 design)");

    restoreDrivenPins();
}

// ============================================================
void setup() {
    pinMode(PIN_PA, OUTPUT);        digitalWrite(PIN_PA, LOW);
    restoreDrivenPins();
    pinMode(PIN_LORA_CS, OUTPUT);   digitalWrite(PIN_LORA_CS, HIGH);
    pinMode(PIN_LORA_RST, OUTPUT);  digitalWrite(PIN_LORA_RST, HIGH);

    for (size_t i = 0; i < N_SCAN; i++) scanMask |= (1u << SCAN_PINS[i]);

    SPI.setRX(PIN_SPI_MISO);
    SPI.setTX(PIN_SPI_MOSI);
    SPI.setSCK(PIN_SPI_CLK);
    SPI.begin();

    Serial.begin(115200);
    delay(3000);
}

void loop() {
    Serial.print("\r\n\r\n");
    Serial.print("\x1b[1;31m############################################################\x1b[0m\r\n");
    Serial.print("\x1b[1;31m#  RF.Guru LoRa radio chip probe  (v2)                     #\x1b[0m\r\n");
    Serial.print("\x1b[1;31m############################################################\x1b[0m\r\n");
    info("SPI: MISO=GP%d SCK=GP%d MOSI=GP%d  CS=GP%d  RST=GP%d",
         PIN_SPI_MISO, PIN_SPI_CLK, PIN_SPI_MOSI, PIN_LORA_CS, PIN_LORA_RST);

    busyKnown = false;
    uint8_t ver = 0, sync[2] = { 0, 0 }, status = 0;
    ChipKind chip = detectChip(ver, sync, status);

    hdr("Chip identification");
    info("  SX127x RegVersion(0x42)   = 0x%02X   (0x12 = SX1276/77/78/79)", ver);
    info("  SX126x GetStatus()        = 0x%02X   (chipMode=%d cmdStatus=%d)",
         status, (status >> 4) & 0x07, (status >> 1) & 0x07);
    info("  SX126x SyncWord(0x0740)   = 0x%02X%02X (0x1424 = SX126x reset value)",
         sync[0], sync[1]);

    switch (chip) {
        case CHIP_SX127X:
            ok("\r\n>>> DETECTED: SX127x / RFM9x (SX1276 family) <<<");
            gpioMap();
            dumpSx127x();
            break;

        case CHIP_SX126X:
            ok("\r\n>>> DETECTED: SX126x (SX1261 / SX1262 / SX1268) <<<");
            verifyReset();
            dumpSx126x();
            gpioMap();
            findBusyPin();
            findDioPins();
            findDio3ViaTcxo();
            testTcxo();
            break;

        default:
            warn("\r\n>>> NO RADIO DETECTED <<<");
            gpioMap();
            break;
    }

    hdr("done - repeating in 20 s");
    delay(20000);
}
