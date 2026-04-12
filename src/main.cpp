// ============================================================
// RF.Guru LoRa 433 APRS Tracker (SmartBeaconing Edition)
// Converted from CircuitPython to Arduino C++ for RP2040
// RF.Guru - ON6URE
// ============================================================

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <FatFS.h>
#include <EEPROM.h>
#include <SPI.h>
#include <Wire.h>
#include <RadioLib.h>
#include <TinyGPSPlus.h>
#include <Adafruit_BME680.h>
#include <Adafruit_SHTC3.h>
#include <hardware/watchdog.h>

#include "pins.h"
#include "config_file.h"
#include "aprs.h"
#include "smartbeacon.h"

// ============================================================
// VERSION
// ============================================================

#define VERSION "RF.Guru_LoRa433APRSTracker 2.0-SB"

// ============================================================
// USB Mass Storage
// ============================================================

Adafruit_USBD_MSC usb_msc;

static int32_t msc_read_cb(uint32_t lba, void *buffer, uint32_t bufsize);
static int32_t msc_write_cb(uint32_t lba, uint8_t *buffer, uint32_t bufsize);
static void msc_flush_cb(void);
static bool fs_changed = false;

// ============================================================
// Global Objects
// ============================================================

TrackerConfig cfg;

// LoRa via RadioLib (SX1276)
SPIClassRP2040 loraSPI(spi0, PIN_SPI_MISO, PIN_LORA_CS, PIN_SPI_CLK, PIN_SPI_MOSI);
// Note: RadioLib wants CS and RST as Module pins
SX1276 radio = new Module(PIN_LORA_CS, RADIOLIB_NC, PIN_LORA_RST, RADIOLIB_NC, loraSPI);

// GPS
TinyGPSPlus gps;
SerialUART gpsSerial(uart0, PIN_GPS_TX, PIN_GPS_RX);

// I2C Sensors
Adafruit_BME680 bme680;
Adafruit_SHTC3 shtc3;
bool hasBME680 = false;
bool hasSHTC3 = false;
bool skipFirstBME680 = true;

// SmartBeacon + Jitter Lock
SmartBeacon sb;
JitterLock jitter;

// ============================================================
// Telemetry state
// ============================================================

static uint16_t sequence = 0;
static unsigned long lastDebugPrint = 0;
static unsigned long lastVoltageWarning = 0;
static unsigned long lastMetadataSend = 0;
static bool metadataForced = true;  // force metadata soon after boot

// GPS LED state
static bool gpsLock = false;
static unsigned long gpsLastBlink = 0;
static unsigned long gpsPulseUntil = 0;
static unsigned long gpsLastNoFixLog = 0;
static unsigned long gpsFixSince = 0;
static bool gpsFixSinceValid = false;
static unsigned long gpsNoFixSince = 0;
static bool gpsNoFixSinceValid = false;

// LoRa header for APRS
static const uint8_t LORA_HEADER[] = { 0x3C, 0xFF, 0x01 };

// ============================================================
// Telemetry metadata strings
// ============================================================

static String metaPARM;
static String metaUNIT;
static String metaEQNS;

static void buildMetadata() {
    metaPARM = "PARM.Satelites";
    metaUNIT = "UNIT.Nr";
    metaEQNS = "EQNS.0,1,0";

    if (cfg.voltage) {
        metaPARM += ",Battery";
        metaUNIT += ",Vdc";
        metaEQNS += ",0,0.01,0";
    }

    if (hasBME680 || hasSHTC3) {
        metaPARM += ",Temp,Hum";
        metaUNIT += ",deg.C,%";
        metaEQNS += ",0,0.02,-50,0,1,0";
    }
}

// ============================================================
// Voltage Reading
// ============================================================

static float getVoltage() {
    int pin = cfg.hasPa ? PIN_ADC_PA : PIN_ADC_NOPA;
    int raw = analogRead(pin);
    if (cfg.hasPa) {
        return ((raw * 3.3f) / 65536.0f) + 10.6f + 1.4f;
    }
    return ((raw * 3.3f) / 65536.0f) * 2.0f;
}

// ============================================================
// LoRa Send
// ============================================================

static void loraSendText(const char *text) {
    watchdog_update();
    digitalWrite(PIN_LED_LORA, HIGH);

    if (cfg.hasPa) {
        digitalWrite(PIN_PA, HIGH);
        delay(250);
    }

    // Build packet: header + text
    int textLen = strlen(text);
    uint8_t *pkt = (uint8_t *)malloc(3 + textLen);
    if (pkt) {
        memcpy(pkt, LORA_HEADER, 3);
        memcpy(pkt + 3, text, textLen);
        radio.transmit(pkt, 3 + textLen);
        free(pkt);
    }

    if (cfg.hasPa) {
        delay(100);
        digitalWrite(PIN_PA, LOW);
    }

    digitalWrite(PIN_LED_LORA, LOW);
    watchdog_update();
}

// ============================================================
// GPS LED + Lock Logic
// ============================================================

static bool updateGpsLed(bool hasFix) {
    unsigned long now = millis();

    // Lock/unlock latch with hold timers
    if (hasFix) {
        gpsNoFixSinceValid = false;
        if (!gpsLock) {
            if (!gpsFixSinceValid) {
                gpsFixSince = now;
                gpsFixSinceValid = true;
            }
            if ((now - gpsFixSince) >= (unsigned long)(cfg.gpsLockHold * 1000)) {
                gpsLock = true;
                gpsFixSinceValid = false;
                Serial.println("[GPS] FIX acquired");
            }
        } else {
            gpsFixSinceValid = false;
        }
    } else {
        gpsFixSinceValid = false;
        if (!gpsNoFixSinceValid) {
            gpsNoFixSince = now;
            gpsNoFixSinceValid = true;
        }
        if (gpsLock && (now - gpsNoFixSince) >= (unsigned long)(cfg.gpsUnlockHold * 1000)) {
            gpsLock = false;
            gpsLastBlink = now - (unsigned long)(cfg.gpsBlinkInterval * 1000);
            gpsPulseUntil = 0;
            Serial.println("[GPS] FIX lost");
        }
    }

    // LED output
    if (gpsLock) {
        digitalWrite(PIN_LED_GPS, HIGH);
    } else {
        if ((now - gpsLastBlink) >= (unsigned long)(cfg.gpsBlinkInterval * 1000)) {
            gpsLastBlink = now;
            gpsPulseUntil = now + (unsigned long)(cfg.gpsBlinkPulse * 1000);
            digitalWrite(PIN_LED_GPS, HIGH);
        } else if (now >= gpsPulseUntil) {
            digitalWrite(PIN_LED_GPS, LOW);
        }
    }

    // Log while no fix
    if (!hasFix) {
        if ((now - gpsLastNoFixLog) >= (unsigned long)(cfg.gpsNoFixLogInterval * 1000)) {
            gpsLastNoFixLog = now;
            Serial.println("[GPS] No GPS FIX... acquiring lock");
        }
    }

    return hasFix;
}

// ============================================================
// Send Metadata
// ============================================================

static void sendMetadata() {
    lastMetadataSend = millis();
    metadataForced = false;

    Serial.println("[TX] Sending telemetry metadata...");

    String metas[] = { metaPARM, metaUNIT, metaEQNS };
    for (int i = 0; i < 3; i++) {
        watchdog_update();
        char frame[200];
        char padCall[10];
        snprintf(padCall, sizeof(padCall), "%-9s", cfg.callsign);
        snprintf(frame, sizeof(frame), "%s>APRFGT::%s:%s",
                 cfg.callsign, padCall, metas[i].c_str());
        Serial.printf("[TX] META: %s\n", frame);
        loraSendText(frame);
        delay(150);
    }
}

// ============================================================
// Send Voltage Alert
// ============================================================

static void sendVoltageAlert(int vx100) {
    float v = vx100 / 100.0f;
    char frame[200];
    char padCall[10];
    snprintf(padCall, sizeof(padCall), "%-9s", cfg.triggerVoltageCall);
    snprintf(frame, sizeof(frame), "%s>APRFGT::%s:Low Voltage (%.2fV) detected!",
             cfg.callsign, padCall, v);
    Serial.printf("[TX] VOLT ALERT: %s\n", frame);
    loraSendText(frame);
}

// ============================================================
// USB MSC Callbacks
// ============================================================

static int32_t msc_read_cb(uint32_t lba, void *buffer, uint32_t bufsize) {
    return fatfs.readSector(lba, (uint8_t *)buffer, bufsize / 512) ? bufsize : -1;
}

static int32_t msc_write_cb(uint32_t lba, uint8_t *buffer, uint32_t bufsize) {
    fs_changed = true;
    return fatfs.writeSector(lba, buffer, bufsize / 512) ? bufsize : -1;
}

static void msc_flush_cb(void) {
    fatfs.syncDevice();
    fs_changed = true;
}

// ============================================================
// SETUP
// ============================================================

void setup() {
    // Watchdog: 5 seconds
    watchdog_enable(5000, true);
    watchdog_update();

    // LEDs
    pinMode(PIN_LED_PWR, OUTPUT);
    pinMode(PIN_LED_GPS, OUTPUT);
    pinMode(PIN_LED_LORA, OUTPUT);
    digitalWrite(PIN_LED_PWR, HIGH);
    digitalWrite(PIN_LED_GPS, LOW);
    digitalWrite(PIN_LED_LORA, LOW);

    // PA off
    pinMode(PIN_PA, OUTPUT);
    digitalWrite(PIN_PA, LOW);

    // I2C power off
    pinMode(PIN_I2C_PWR, OUTPUT);
    digitalWrite(PIN_I2C_PWR, LOW);

    // GPS reset
    pinMode(PIN_GPS_RST, OUTPUT);
    digitalWrite(PIN_GPS_RST, LOW);
    delay(100);
    watchdog_update();
    digitalWrite(PIN_GPS_RST, HIGH);
    delay(100);
    watchdog_update();

    // USB Serial
    Serial.begin(115200);

    // Init FAT filesystem
    fatfs.begin();
    watchdog_update();

    // USB Mass Storage
    usb_msc.setID("RF.Guru", "APRSTracker", "2.0");
    usb_msc.setReadWriteCallback(msc_read_cb, msc_write_cb, msc_flush_cb);
    usb_msc.setCapacity(fatfs.sectorCount(), 512);
    usb_msc.setUnitReady(true);
    usb_msc.begin();

    delay(2000);
    watchdog_update();

    Serial.println();
    Serial.println("============================================================");
    Serial.printf(" -- Tracker Booted: %s\n", VERSION);
    Serial.println("============================================================");

    // Load config from filesystem
    if (!configLoad(cfg)) {
        Serial.println("[CONFIG] Using defaults");
        configSetDefaults(cfg);
        configApplyProfile(cfg);
    }

    Serial.printf("[CONFIG] Callsign: %s  Profile: %s\n", cfg.callsign, cfg.profile);
    Serial.println("[CONFIG] SmartBeaconing: ON");

    // ADC setup
    analogReadResolution(16);

    // EEPROM for sequence persistence
    EEPROM.begin(4);
    uint16_t stored = EEPROM.read(0) | (EEPROM.read(1) << 8);
    if (stored > 0 && stored <= 8191) {
        sequence = stored;
    }
    Serial.printf("[SEQ] Start at: %d\n", sequence);

    // LoRa init
    Serial.println("[LORA] Initializing...");
    loraSPI.begin();

    int state = radio.begin(cfg.loraFrequency, 125.0, 12, 5, RADIOLIB_SX127X_SYNC_WORD, cfg.power);
    if (state != RADIOLIB_ERR_NONE) {
        Serial.printf("[LORA] INIT ERROR: %d\n", state);
        while (true) {
            watchdog_update();
            delay(1000);
        }
    }
    // LoRa APRS settings: explicit header, CRC on
    radio.explicitHeader();
    radio.setCRC(true);
    Serial.printf("[LORA] OK (freq=%.3f MHz, pwr=%d dBm)\n", cfg.loraFrequency, cfg.power);

    // GPS init
    Serial.println("[GPS] Initializing...");
    gpsSerial.begin(9600);

    // Set GPS to 1Hz
    const uint8_t gps1hz[] = {
        0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
        0xE8, 0x03, 0x01, 0x00, 0x01, 0x00,
        0x01, 0x39
    };
    gpsSerial.write(gps1hz, sizeof(gps1hz));
    delay(100);

    // Disable UBX messages
    const uint8_t disableUbx[] = {
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xB9,
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0xC0
    };
    gpsSerial.write(disableUbx, sizeof(disableUbx));
    delay(100);
    watchdog_update();

    // I2C sensors
    if (cfg.i2cEnabled) {
        Serial.println("[I2C] Initializing sensors...");
        digitalWrite(PIN_I2C_PWR, HIGH);
        watchdog_update();
        delay(2000);
        watchdog_update();

        Wire.setSDA(PIN_I2C_SDA);
        Wire.setSCL(PIN_I2C_SCL);
        Wire.begin();

        if (strcasecmp(cfg.i2cDevice, "BME680") == 0) {
            if (bme680.begin(0x77, &Wire)) {
                hasBME680 = true;
                bme680.setTemperatureOversampling(BME680_OS_8X);
                bme680.setHumidityOversampling(BME680_OS_2X);
                bme680.setPressureOversampling(BME680_OS_4X);
                Serial.println("[I2C] BME680 OK");
            } else {
                Serial.println("[I2C] BME680 not found");
            }
        } else if (strcasecmp(cfg.i2cDevice, "SHTC3") == 0) {
            if (shtc3.begin(&Wire)) {
                hasSHTC3 = true;
                Serial.println("[I2C] SHTC3 OK");
            } else {
                Serial.println("[I2C] SHTC3 not found");
            }
        }

        if (!hasBME680 && !hasSHTC3) {
            digitalWrite(PIN_I2C_PWR, LOW);
        }
    }

    // Build telemetry metadata
    buildMetadata();

    // Init beacon engine
    sb.reset();
    jitter.reset();

    // Init GPS LED timing
    gpsLastBlink = millis() - (unsigned long)(cfg.gpsBlinkInterval * 1000);

    Serial.println("[TRACKER] Ready, entering main loop");
    watchdog_update();
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
    watchdog_update();

    // Feed GPS data
    while (gpsSerial.available()) {
        gps.encode(gpsSerial.read());
    }

    // Check if we have a valid fix
    bool hasFix = gps.location.isValid() && gps.location.isUpdated();

    // Update GPS LED (always)
    if (!updateGpsLed(hasFix)) {
        delay(50);
        return;
    }

    // Need valid date/time
    if (!gps.date.isValid() || gps.date.year() < 2023) {
        if (cfg.fullDebug) {
            Serial.println("[GPS] Waiting for valid date/time...");
        }
        delay(50);
        return;
    }

    // Read GPS data
    float rawLat = gps.location.lat();
    float rawLon = gps.location.lng();
    float altM = gps.altitude.isValid() ? gps.altitude.meters() : -1;
    float heading = gps.course.isValid() ? gps.course.deg() : -1;
    float speedKmh = gps.speed.isValid() ? gps.speed.kmph() : -1;

    // Stabilize stationary jitter
    float lat = rawLat;
    float lon = rawLon;
    bool startedMoving = false;
    float movedM = 0;
    jitter.stabilize(lat, lon, speedKmh >= 0 ? speedKmh : 0,
                     cfg.sbStationarySpeed, cfg.sbStationaryDistance,
                     cfg.sbStationaryExitCount, startedMoving, movedM);

    // Debug print every 5 seconds
    unsigned long nowMs = millis();
    if ((nowMs - lastDebugPrint) >= 5000) {
        lastDebugPrint = nowMs;
        Serial.printf("[FIX] LAT=%.6f LON=%.6f SPD=%.1fkm/h HDG=%.0f ST=%s JIT=%.0fm\n",
                      lat, lon,
                      speedKmh >= 0 ? speedKmh : 0,
                      heading >= 0 ? heading : 0,
                      jitter.isStationary() ? "Y" : "N",
                      movedM);
    }

    // Determine whether to beacon
    float sbSpeed = jitter.isStationary() ? 0.0f : (speedKmh >= 0 ? speedKmh : 0);
    bool sendBeacon = startedMoving || sb.shouldBeacon(
        lat, lon, sbSpeed, heading,
        cfg.sbStationarySpeed, cfg.sbSlowSpeed, cfg.sbFastSpeed,
        cfg.sbSlowRate, cfg.sbFastRate,
        cfg.sbTurnThreshold, cfg.sbTurnSlope, cfg.sbHeadingFilter);

    // Voltage monitoring
    int pendingVoltAlert = -1;
    if (cfg.voltage && cfg.triggerVoltage) {
        int vx100 = (int)(getVoltage() * 100 + 0.5f);
        if (vx100 >= 1000 && vx100 <= cfg.triggerVoltageLevel) {
            if ((nowMs - lastVoltageWarning) > (unsigned long)cfg.triggerVoltageKeepalive * 1000UL) {
                lastVoltageWarning = nowMs;
                pendingVoltAlert = vx100;
                Serial.printf("[VOLT] LOW: %.2fV\n", vx100 / 100.0f);
            }
        }
    }

    // Daily metadata timer
    bool metadataDue = metadataForced || ((nowMs - lastMetadataSend) >= 86400000UL);

    // Nothing to do?
    if (!sendBeacon && pendingVoltAlert < 0 && !metadataDue) {
        delay(50);
        return;
    }

    // Metadata only
    if (metadataDue && !sendBeacon && pendingVoltAlert < 0) {
        sendMetadata();
        delay(50);
        return;
    }

    // ============================================================
    // SEND POSITION BEACON
    // ============================================================
    if (sendBeacon) {
        // Timestamp
        char ts[8];
        aprsTimestamp(ts, 'z', gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second());

        // Position
        char aprsPos[20];
        float spdForAprs = speedKmh >= 0 ? speedKmh : -1;
        float hdgForAprs = heading >= 0 ? heading : -1;
        aprsPosition(aprsPos, lat, lon, spdForAprs, hdgForAprs, cfg.symbol);

        // Sequence
        sequence = (sequence + 1) % 8192;
        EEPROM.write(0, sequence & 0xFF);
        EEPROM.write(1, (sequence >> 8) & 0xFF);
        EEPROM.commit();

        // Build comment with telemetry
        char comment[128];
        int cpos = 0;
        cpos += snprintf(comment + cpos, sizeof(comment) - cpos, "%s|", cfg.comment);

        char b91[8];
        base91Encode(b91, sequence);
        cpos += snprintf(comment + cpos, sizeof(comment) - cpos, "%s", b91);

        int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;
        base91Encode(b91, sats);
        cpos += snprintf(comment + cpos, sizeof(comment) - cpos, "%s", b91);

        if (cfg.voltage) {
            int batx100 = (int)(getVoltage() * 100 + 0.5f);
            base91Encode(b91, batx100);
            cpos += snprintf(comment + cpos, sizeof(comment) - cpos, "%s", b91);
        }

        // Temperature / Humidity
        if (hasSHTC3) {
            sensors_event_t hum, temp;
            shtc3.getEvent(&hum, &temp);
            float tC = temp.temperature;
            float rH = hum.relative_humidity;
            int tCode = (int)((tC / 2.0f + 25.0f) * 100);
            int hCode = (int)(rH + 0.5f);
            base91Encode(b91, tCode);
            cpos += snprintf(comment + cpos, sizeof(comment) - cpos, "%s", b91);
            base91Encode(b91, hCode);
            cpos += snprintf(comment + cpos, sizeof(comment) - cpos, "%s", b91);
        } else if (hasBME680) {
            if (skipFirstBME680) {
                skipFirstBME680 = false;
                bme680.performReading();
            } else {
                if (bme680.performReading()) {
                    float tC = bme680.temperature + (float)cfg.bme680TempOffset;
                    float rH = bme680.humidity;
                    int tCode = (int)((tC / 2.0f + 25.0f) * 100);
                    int hCode = (int)(rH + 0.5f);
                    base91Encode(b91, tCode);
                    cpos += snprintf(comment + cpos, sizeof(comment) - cpos, "%s", b91);
                    base91Encode(b91, hCode);
                    cpos += snprintf(comment + cpos, sizeof(comment) - cpos, "%s", b91);
                }
            }
        }

        cpos += snprintf(comment + cpos, sizeof(comment) - cpos, "|");

        if (altM >= 0) {
            int feet = (int)(altM * 3.2808399f);
            cpos += snprintf(comment + cpos, sizeof(comment) - cpos, "/A=%06d", feet);
        }

        // Build full APRS frame
        char frame[300];
        snprintf(frame, sizeof(frame), "%s>APRFGT:@%s%s%s",
                 cfg.callsign, ts, aprsPos, comment);

        Serial.printf("[TX] %s\n", frame);
        loraSendText(frame);
        sb.updateAfterBeacon(lat, lon);
    }

    // Metadata if due
    if (metadataDue) {
        sendMetadata();
    }

    // Voltage alert if pending
    if (pendingVoltAlert >= 0) {
        sendVoltageAlert(pendingVoltAlert);
    }

    delay(50);
}
