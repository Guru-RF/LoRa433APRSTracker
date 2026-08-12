// ============================================================
// RF.Guru CW power test
//
// TRANSMITS A CONTINUOUS CARRIER. Antenna or dummy load required, and a
// callsign you are licensed to use. Each step is short, but this is an
// unmodulated carrier on 433.775 MHz, not a packet.
//
// Why it exists: the module's measured output came in about 4 dB under
// the MiniF27 datasheet's drive table, and a LoRa burst is a poor thing
// to measure - it lasts a couple of seconds and most meters want a
// steady signal. This holds CW at each setting long enough to read.
//
// It also separates the two candidate explanations:
//
//   RadioLib's SX1262::setOutputPower(9) does NOT program "level 9". It
//   looks up an optimised PA configuration - paDutyCycle=1, hpMax=2,
//   SetTxParams=20 - tuned to yield +9 dBm from a bare SX1262 into 50
//   ohm. NiceRF's "Chip Output Level" column is far more likely to mean
//   SetTxParams=9 with the standard PA configuration (paDutyCycle=4,
//   hpMax=7). Those are different drive conditions into their PA.
//
// Steps 1 and 2 below are the same nominal drive under each
// interpretation, so the meter reading tells you which one NiceRF
// characterised - and whether the gap is the PA config at all.
//
// While a step is running, also measure VCC_PA (module pin 7, the
// cathode side of the series diode) with a DMM. If that rail is sagging
// under the PA current, it explains the shortfall on its own and no
// amount of drive tuning will fix it.
// ============================================================

#include <Arduino.h>
#include <SPI.h>

#include "config_file.h"
#include "pins.h"
#include "radio.h"

static TrackerConfig cfg;
static SPISettings loraSpi(2000000, MSBFIRST, SPI_MODE0);

static void waitBusy(uint32_t timeoutUs = 20000) {
    uint32_t t0 = micros();
    while (digitalRead(PIN_LORA_BUSY)) {
        if ((uint32_t)(micros() - t0) > timeoutUs) return;
    }
}

static void cmd(uint8_t opcode, const uint8_t *data, size_t n) {
    SPI.beginTransaction(loraSpi);
    digitalWrite(PIN_LORA_CS, LOW);
    SPI.transfer(opcode);
    for (size_t i = 0; i < n; i++) SPI.transfer(data[i]);
    digitalWrite(PIN_LORA_CS, HIGH);
    SPI.endTransaction();
    waitBusy();
}

struct Step {
    const char *label;
    uint8_t paDutyCycle;
    uint8_t hpMax;
    int8_t  txParams;
};

// ASCENDING drive sweep.
//
// The descending sweep established two things: output tracks drive
// monotonically, and RadioLib's optimised entry for +9 dBm gave more
// than the standard PA config at SetTxParams=9 - so the datasheet's
// "Chip Output Level" column is not the standard config, and the
// firmware's current setting is already the better of the two. What it
// did not establish is why the whole curve sits ~3.3 dB below the
// datasheet.
//
// Two candidates remain: the PA is starved by a sagging VCC_PA, or it
// is simply under-driven. Driving it harder separates them without
// needing a DMM. If output plateaus, the supply is the ceiling. If it
// keeps climbing toward the rated 29 dBm, it wanted more drive.
//
// These entries are RadioLib's paOptTable[target+9] for targets +9,
// +13, +16, +19 and +22 dBm, read from SX1262.cpp. Steps 2 onward
// exceed NiceRF's documented drive column, which stops at 9.
static const Step STEPS[] = {
    { "1  optimised target  +9 dBm  (dc=1 hp=2 tx=20)  <- today's setting", 1, 2, 20 },
    { "2  optimised target +13 dBm  (dc=1 hp=4 tx=19)",                     1, 4, 19 },
    { "3  optimised target +16 dBm  (dc=2 hp=5 tx=19)",                     2, 5, 19 },
    { "4  optimised target +19 dBm  (dc=3 hp=5 tx=22)",                     3, 5, 22 },
    { "5  optimised target +22 dBm  (dc=4 hp=7 tx=22)",                     4, 7, 22 },
};

static const uint32_t STEP_MS = 10000;
static const int      PASSES  = 2;

// With the Powerpole connected there is no USB, so the board has to say
// what it is doing on its own: the GPS LED blinks the step number, then
// the LoRa LED stays solid for as long as the carrier is on.
static void blinkStep(int n) {
    for (int i = 0; i < n; i++) {
        digitalWrite(PIN_LED_GPS, HIGH);
        delay(250);
        digitalWrite(PIN_LED_GPS, LOW);
        delay(250);
    }
    delay(1000);
}

void setup() {
    pinMode(PIN_PA, OUTPUT);        digitalWrite(PIN_PA, LOW);
    pinMode(PIN_I2C_PWR, OUTPUT);   digitalWrite(PIN_I2C_PWR, LOW);
    pinMode(PIN_LED_PWR, OUTPUT);   digitalWrite(PIN_LED_PWR, HIGH);
    pinMode(PIN_LED_GPS, OUTPUT);   digitalWrite(PIN_LED_GPS, LOW);
    pinMode(PIN_LED_LORA, OUTPUT);  digitalWrite(PIN_LED_LORA, LOW);

    Serial.begin(115200);

    // No USB when the Powerpole is in, so nobody is reading this. Give
    // the operator a few seconds at the meter before the first carrier;
    // all three LEDs flash together to mark the start.
    for (int i = 0; i < 5; i++) {
        digitalWrite(PIN_LED_GPS, HIGH);  digitalWrite(PIN_LED_LORA, HIGH);
        delay(150);
        digitalWrite(PIN_LED_GPS, LOW);   digitalWrite(PIN_LED_LORA, LOW);
        delay(850);
    }

    configSetDefaults(cfg);
    configApplyProfile(cfg);
}

void loop() {
    // A bounded number of passes. An unmodulated carrier that re-keys
    // forever is not something to leave running on a bench you have
    // walked away from; power-cycle to repeat.
    static int pass = 0;
    if (pass >= PASSES) {
        // Idle heartbeat on the power LED so it is obvious the sweep has
        // finished rather than died mid-carrier.
        digitalWrite(PIN_LED_PWR, LOW);  delay(100);
        digitalWrite(PIN_LED_PWR, HIGH); delay(1900);
        return;
    }
    pass++;

    Serial.print("\r\n\x1b[1;31m=== CW power test - TRANSMITTING A CARRIER ===\x1b[0m\r\n");

    char err[96] = "";
    if (!TrackerRadio::begin(cfg, err, sizeof(err))) {
        Serial.printf("begin failed: %s\r\n", err);
        delay(15000);
        return;
    }
    if (TrackerRadio::chip() != RADIO_CHIP_SX126X) {
        Serial.println("not an SX126x - this test is SX126x only");
        delay(15000);
        return;
    }

    Serial.printf("pass %d/%d, %.3f MHz, %lu s per step\r\n",
                  pass, PASSES, cfg.loraFrequency, (unsigned long)(STEP_MS / 1000));
    Serial.println("measure forward power at each step; also probe VCC_PA with a DMM\r\n");

    // The module's amplifier needs its supply up before any carrier.
    digitalWrite(PIN_PA, HIGH);
    delay(250);

    for (size_t i = 0; i < sizeof(STEPS) / sizeof(STEPS[0]); i++) {
        const Step &s = STEPS[i];

        blinkStep((int)i + 1);

        const uint8_t paCfg[4] = { s.paDutyCycle, s.hpMax, 0x00, 0x01 };  // deviceSel=SX1262
        cmd(0x95, paCfg, 4);                                              // SetPaConfig

        const uint8_t txp[2] = { (uint8_t)s.txParams, 0x04 };             // 200 us ramp
        cmd(0x8E, txp, 2);                                                // SetTxParams

        Serial.printf("\x1b[1;33m%s\x1b[0m\r\n", s.label);
        digitalWrite(PIN_LED_LORA, HIGH);       // solid = carrier is on
        cmd(0xD1, nullptr, 0);                                            // SetTxContinuousWave

        for (uint32_t t = 0; t < STEP_MS; t += 1000) {
            Serial.printf("   carrier on, %lu s left\r\n",
                          (unsigned long)((STEP_MS - t) / 1000));
            delay(1000);
        }

        const uint8_t stdby[1] = { 0x00 };
        cmd(0x80, stdby, 1);                                              // SetStandby(RC)
        digitalWrite(PIN_LED_LORA, LOW);
        Serial.println("   carrier off\r\n");
        delay(2000);
    }

    delay(100);
    digitalWrite(PIN_PA, LOW);

    Serial.println("\x1b[1;32mpass done - carrier off, PA disabled.\x1b[0m");
    Serial.println("Rising then flat = supply-limited. Rising all the way to");
    Serial.println("step 5 = the PA simply wanted more drive.");
}
