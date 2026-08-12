// ============================================================
// RF.Guru radio bring-up self-test
//
// Compiles the production src/radio.cpp and runs the exact boot path
// the tracker firmware uses - autodetect, oscillator probe, RadioLib
// configuration - then reads the settings back out of the chip.
//
// By default it stops short of TrackerRadio::send(), so nothing is
// transmitted and the external PA is never enabled. Use it to validate
// a new board revision before letting it key up.
//
// Building with -DRADIOTEST_TX adds a single on-air transmission, once
// per boot, of one APRS frame at the configured frequency and power.
// Only do that with an antenna or dummy load fitted and a callsign you
// are licensed to use. The external PA stays off either way.
// ============================================================

#include <Arduino.h>

#include "config_file.h"
#include "pins.h"
#include "radio.h"

static TrackerConfig cfg;

static void line(const char *fmt, ...) {
    char b[256];
    va_list ap; va_start(ap, fmt); vsnprintf(b, sizeof(b), fmt, ap); va_end(ap);
    Serial.printf("%s\r\n", b);
}

void setup() {
    // Anything that could key the amplifier, off and staying off.
    pinMode(PIN_PA, OUTPUT);        digitalWrite(PIN_PA, LOW);
    pinMode(PIN_I2C_PWR, OUTPUT);   digitalWrite(PIN_I2C_PWR, LOW);
    pinMode(PIN_LED_PWR, OUTPUT);   digitalWrite(PIN_LED_PWR, HIGH);
    pinMode(PIN_LED_GPS, OUTPUT);   digitalWrite(PIN_LED_GPS, LOW);
    pinMode(PIN_LED_LORA, OUTPUT);  digitalWrite(PIN_LED_LORA, LOW);

    Serial.begin(115200);
    delay(3000);

    configSetDefaults(cfg);
    configApplyProfile(cfg);
}

void loop() {
    Serial.print("\r\n\x1b[1;31m=== radio bring-up self-test (no TX) ===\x1b[0m\r\n");

    RadioChip probed = TrackerRadio::probe();
    line("probe()            : %s",
         probed == RADIO_CHIP_SX126X ? "SX126x" :
         probed == RADIO_CHIP_SX127X ? "SX127x" : "NONE");

    char err[96] = "";
    if (!TrackerRadio::begin(cfg, err, sizeof(err))) {
        Serial.printf("\x1b[1;31mbegin() FAILED: %s\x1b[0m\r\n", err);
        delay(15000);
        return;
    }

    line("chip               : %s", TrackerRadio::chipName());
    line("frequency          : %.3f MHz", cfg.loraFrequency);
    line("power requested    : %d dBm", cfg.power);
    line("power applied      : %d dBm", TrackerRadio::appliedPower());
    if (TrackerRadio::chip() == RADIO_CHIP_SX126X) {
        float v = TrackerRadio::tcxoVoltage();
        if (v > 0.0f) line("clock              : TCXO on DIO3 @ %.1f V", v);
        else          line("clock              : crystal");
    }

    char info[192];
    TrackerRadio::describe(info, sizeof(info));
    line("readback           : %s", info);

    // Time on air is derived from SF, BW, coding rate, preamble, header
    // and CRC, so matching values across both chips is a good proxy for
    // "these two radios put the same thing on the air".
    line("");
    line("time on air by payload length (SF12 / BW125 / CR4:5 expected):");
    static const size_t lens[] = { 16, 32, 64, 96, 128 };
    for (size_t i = 0; i < sizeof(lens) / sizeof(lens[0]); i++) {
        line("  %3u bytes -> %lu ms", (unsigned)lens[i],
             (unsigned long)TrackerRadio::timeOnAirMs(lens[i]));
    }

#ifdef RADIOTEST_TX
    static bool transmitted = false;
    if (!transmitted) {
        transmitted = true;

        // Sweep the amplifier from gentle to full so one run shows both
        // that the path works and whether the supply sags at the top of
        // the range - the module pulls 460 mA at drive level 9.
        static const int8_t LEVELS[] = { 0, 4, 9 };

        for (size_t i = 0; i < sizeof(LEVELS) / sizeof(LEVELS[0]); i++) {
            cfg.power = LEVELS[i];
            char err2[96] = "";
            if (!TrackerRadio::begin(cfg, err2, sizeof(err2))) {
                Serial.printf("\x1b[1;31m  re-begin failed: %s\x1b[0m\r\n", err2);
                break;
            }

            int16_t deci = TrackerRadio::antennaPowerDeciDbm();

            // Same framing as the tracker: 3-byte LoRa-APRS header, then
            // the ASCII frame.
            static const uint8_t header[] = { 0x3C, 0xFF, 0x01 };
            char frame[128];
            snprintf(frame, sizeof(frame), "%s>APRFGT::%-9s:selftest pwr %d",
                     cfg.callsign, cfg.callsign, TrackerRadio::appliedPower());

            uint8_t packet[sizeof(header) + sizeof(frame)];
            size_t frameLen = strlen(frame);
            memcpy(packet, header, sizeof(header));
            memcpy(packet + sizeof(header), frame, frameLen);
            size_t total = sizeof(header) + frameLen;

            line("");
            line("TX %u/3: drive %d -> %d.%d dBm at the antenna, %u bytes, ~%lu ms",
                 (unsigned)(i + 1), TrackerRadio::appliedPower(),
                 deci / 10, abs(deci % 10), (unsigned)total,
                 (unsigned long)TrackerRadio::timeOnAirMs(total));

            // Mirror the tracker's amplifier sequencing exactly. This is
            // not cosmetic: if GP2 gates the module's PA supply,
            // transmitting with it low leaves BUSY asserted through a PA
            // ramp that never completes, which hangs RadioLib's SetTx.
            if (cfg.hasPa) {
                digitalWrite(PIN_PA, HIGH);
                delay(250);
            }

            uint32_t t0 = millis();
            bool sent  = TrackerRadio::send(packet, total);
            uint32_t elapsed = millis() - t0;

            if (cfg.hasPa) {
                delay(100);
                digitalWrite(PIN_PA, LOW);
            }

            if (sent) Serial.printf("\x1b[1;32m  TX OK in %lu ms\x1b[0m\r\n", (unsigned long)elapsed);
            else      Serial.printf("\x1b[1;31m  TX FAILED after %lu ms\x1b[0m\r\n", (unsigned long)elapsed);

            char after[192];
            TrackerRadio::describe(after, sizeof(after));
            line("  readback: %s", after);
            delay(500);
        }
    } else {
        line("");
        line("(already transmitted this boot - reset to send again)");
    }
#endif

    Serial.print("\r\n\x1b[1;32mbring-up OK\x1b[0m\r\n");
    delay(20000);
}
