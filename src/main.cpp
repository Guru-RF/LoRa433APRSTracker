// ============================================================
// RF.Guru LoRa 433 APRS Tracker (SmartBeaconing Edition)
// Converted from CircuitPython to Arduino C++ for RP2040
// RF.Guru - ON6URE
// ============================================================

#include <Arduino.h>
#include <FatFS.h>
#include <FatFSUSB.h>
#include <EEPROM.h>
#include <SPI.h>
#include <SerialPIO.h>
#include <Wire.h>
#include <TinyGPSPlus.h>
#include <Adafruit_BME680.h>
#include <Adafruit_SHTC3.h>
#include <hardware/watchdog.h>

#include "pins.h"
#include "config_file.h"
#include "aprs.h"
#include "smartbeacon.h"
#include "radio.h"

// ============================================================
// VERSION
// ============================================================

#define VERSION "RF.Guru_LoRa433APRSTracker 2.1-SB"

// ============================================================
// ANSI Color helpers (matching Python version)
// ============================================================

// Forward declare gps for timestamp access
extern TinyGPSPlus gps;

// purple: timestamped info (position reports, TX) - uses GPS time when available
static void purple(const char *msg) {
    if (gps.time.isValid() && gps.date.isValid() && gps.date.year() >= 2023) {
        Serial.printf("\x1b[38;5;104m[%02d/%02d/%04d %02d:%02d:%02d] %s\x1b[0m\r\n",
                      gps.date.month(), gps.date.day(), gps.date.year(),
                      gps.time.hour(), gps.time.minute(), gps.time.second(), msg);
    } else {
        unsigned long s = millis() / 1000;
        int h = (s / 3600) % 24, m = (s / 60) % 60, sec = s % 60;
        Serial.printf("\x1b[38;5;104m[%02d:%02d:%02d] %s\x1b[0m\r\n", h, m, sec, msg);
    }
}

// yellow: init steps, warnings
static void yellow(const char *msg) {
    Serial.printf("\x1b[38;5;220m%s\x1b[0m\r\n", msg);
}

// red: errors, alerts, boot banner
static void red(const char *msg) {
    Serial.printf("\x1b[1;5;31m%s\x1b[0m\r\n", msg);
}

// green: success messages
static void green(const char *msg) {
    Serial.printf("\x1b[1;5;32m%s\x1b[0m\r\n", msg);
}

// ============================================================
// Global Objects
// ============================================================

TrackerConfig cfg;

// GPS
TinyGPSPlus gps;
SerialPIO gpsSerial(PIN_GPS_TX, PIN_GPS_RX, 256);

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
static bool metadataForced = true;

// Board revision, read from the GP15 strap at boot
static bool boardV2 = false;

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

    // On a module with its own amplifier the enable line is not
    // optional: if GP2 gates that amplifier's supply, transmitting with
    // it low leaves BUSY asserted through a PA ramp that never
    // completes, and RadioLib waits on BUSY with no timeout. Assert it
    // whenever a PA is present, however the config is written.
    bool needPa = cfg.hasPa || TrackerRadio::hasModulePa();

    if (needPa) {
        digitalWrite(PIN_PA, HIGH);
        delay(250);
        watchdog_update();
    }

    // Both families reject anything over 255 bytes outright, where the
    // old library silently truncated. Truncate here so an overlong
    // comment degrades the frame instead of dropping the beacon.
    const size_t LORA_MAX_PACKET = 255;
    uint8_t packet[LORA_MAX_PACKET];
    size_t textLen = strlen(text);
    if (textLen > LORA_MAX_PACKET - sizeof(LORA_HEADER)) {
        textLen = LORA_MAX_PACKET - sizeof(LORA_HEADER);
    }
    memcpy(packet, LORA_HEADER, sizeof(LORA_HEADER));
    memcpy(packet + sizeof(LORA_HEADER), text, textLen);

    if (!TrackerRadio::send(packet, sizeof(LORA_HEADER) + textLen)) {
        red("TX FAILED");
    }

    watchdog_update();

    if (needPa) {
        delay(100);
        digitalWrite(PIN_PA, LOW);
    }

    digitalWrite(PIN_LED_LORA, LOW);
    watchdog_update();
}

// ============================================================
// GPS bring-up
//
// V1 boards carry a u-blox NEO, which speaks UBX and defaults to 9600.
// V2 boards carry a Zhongke ATGM336H (AT6558) - the part had to change
// because u-blox modules cannot be imported into the assembly factory -
// which speaks CASIC/PCAS and defaults to 115200.
//
// Nothing here tries to work out which one is fitted. The baud rate is
// found by listening, and both command sets are sent unconditionally:
// UBX reaching an ATGM336H is binary noise it discards, and a PCAS
// sentence reaching a u-blox is an unrecognised NMEA line it ignores.
// That is cheaper and more reliable than any board-revision guess - the
// GP15 strap reads low on V1 hardware too, so it identifies nothing.
//
// Both emit $GN.. talker IDs once multiple constellations are in use,
// which TinyGPSPlus parses.
// ============================================================

// Send an NMEA command, appending the checksum, so the command strings
// stay readable here.
static void gpsSendNmea(const char *body) {
    uint8_t cs = 0;
    for (const char *p = body; *p; p++) cs ^= (uint8_t)*p;
    gpsSerial.printf("$%s*%02X\r\n", body, cs);
}

// Listen for the start of two NMEA sentences.
static bool gpsSeesNmea(uint32_t windowMs) {
    uint32_t t0 = millis();
    int dollars = 0;
    while ((millis() - t0) < windowMs) {
        while (gpsSerial.available()) {
            if (gpsSerial.read() == '$' && ++dollars >= 2) return true;
        }
        watchdog_update();
    }
    return false;
}

static bool gpsTryBaud(uint32_t baud, uint32_t windowMs) {
    gpsSerial.end();
    delay(20);
    gpsSerial.begin(baud);
    delay(20);
    while (gpsSerial.available()) gpsSerial.read();     // discard partials
    return gpsSeesNmea(windowMs);
}

// Returns the baud rate the GPS was found on, or 0 if it stayed silent.
static uint32_t gpsBringUp(const TrackerConfig &cfg) {
    uint32_t found = 0;

    if (cfg.gpsBaud > 0) {
        // Pinned by config: use it whether or not anything answers.
        gpsSerial.end();
        delay(20);
        gpsSerial.begin(cfg.gpsBaud);
        found = cfg.gpsBaud;
    } else {
        // Try the rate this board most likely ships with first, so the
        // common case costs one short window. The radio is the honest
        // discriminator here - it was read off the chip a moment ago,
        // whereas the GP15 strap reads low on both revisions. A board
        // that pairs them differently still comes up via the sweep.
        const uint32_t likely =
            (TrackerRadio::chip() == RADIO_CHIP_SX126X) ? 115200 : 9600;
        static const uint32_t rates[] = { 0, 9600, 115200, 38400, 57600, 19200 };
        for (size_t i = 0; i < sizeof(rates) / sizeof(rates[0]); i++) {
            uint32_t baud = (i == 0) ? likely : rates[i];
            if (i > 0 && baud == likely) continue;      // already tried
            if (gpsTryBaud(baud, 1500)) { found = baud; break; }
        }
        if (!found) {
            gpsSerial.end();
            delay(20);
            gpsSerial.begin(9600);
        }
    }

    watchdog_update();

    // UBX-CFG-RATE: 1 Hz (u-blox)
    const uint8_t gps1hz[] = {
        0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
        0xE8, 0x03, 0x01, 0x00, 0x01, 0x00,
        0x01, 0x39
    };
    gpsSerial.write(gps1hz, sizeof(gps1hz));
    delay(100);

    // UBX-CFG-MSG: silence the binary messages (u-blox)
    const uint8_t disableUbx[] = {
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xB9,
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03,
        0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0xC0
    };
    gpsSerial.write(disableUbx, sizeof(disableUbx));
    delay(100);
    watchdog_update();

    // CASIC/PCAS: 1 Hz, and GGA + RMC only (ATGM336H). Trimming the
    // sentence set matters - the default burst can outrun the 256-byte
    // PIO FIFO while a beacon is on the air for several seconds.
    gpsSendNmea("PCAS02,1000");
    delay(100);
    gpsSendNmea("PCAS03,1,0,0,0,1,0,0,0");
    delay(100);

    watchdog_update();
    return found;
}

// ============================================================
// GPS LED + Lock Logic
// ============================================================

static bool updateGpsLed(bool hasFix) {
    unsigned long now = millis();

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
                green("GPS FIX acquired");
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
            yellow("GPS FIX lost");
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
            yellow("No GPS FIX... acquiring lock");
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

    yellow("Sending telemetry metadata...");

    String metas[] = { metaPARM, metaUNIT, metaEQNS };
    for (int i = 0; i < 3; i++) {
        watchdog_update();
        char frame[200];
        char padCall[10];
        snprintf(padCall, sizeof(padCall), "%-9s", cfg.callsign);
        snprintf(frame, sizeof(frame), "%s>APRFGT::%s:%s",
                 cfg.callsign, padCall, metas[i].c_str());
        char buf[256];
        snprintf(buf, sizeof(buf), "TX META: %s", frame);
        purple(buf);
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
    char buf[256];
    snprintf(buf, sizeof(buf), "TX VOLT ALERT: %s", frame);
    red(buf);
    loraSendText(frame);
}

// ============================================================
// USB drive ready callback
// ============================================================

static bool usbDriveReady(uint32_t) {
    return true;
}

// ============================================================
// SETUP
// ============================================================

void setup() {
    // No watchdog during init - enable at end of setup()

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

    // Board revision strap (grounded on V2, floating on V1)
    boardV2 = TrackerRadio::boardIsV2();

    // GPS reset (hold low 1s, then high 1s - same as Python boot.py)
    pinMode(PIN_GPS_RST, OUTPUT);
    digitalWrite(PIN_GPS_RST, LOW);
    delay(1000);
    digitalWrite(PIN_GPS_RST, HIGH);
    delay(1000);

    // USB Serial
    Serial.begin(115200);
    delay(2000);

    Serial.print("\r\n");

    char banner[128];
    snprintf(banner, sizeof(banner), " -- Tracker Booted: %s -=- %s", cfg.callsign, VERSION);
    red(banner);

    // Init FAT filesystem and do all file ops BEFORE starting USB drive
    if (!FatFS.begin()) {
        red("FatFS init failed!");
        while (true) delay(1000);
    }
    fatfs::f_setlabel("APRSTRKR");

    // Load config (creates default if missing)
    if (!configLoad(cfg)) {
        yellow("Using default configuration");
        configSetDefaults(cfg);
        configApplyProfile(cfg);
    }

    // Check for firmware update trigger
    File cfgFile = FatFS.open("/config.txt", "r");
    if (cfgFile) {
        String content;
        while (cfgFile.available()) content += (char)cfgFile.read();
        cfgFile.close();
        content.trim();
        if (content == "firmwareupdate") {
            red("Firmware update requested!");
            FatFS.remove("/config.txt");
            delay(500);
            red("Rebooting into UF2 bootloader...");
            delay(500);
            reset_usb_boot(0, 0);
            while (true);
        }
    }

    // Start USB mass storage (after all file operations)
    FatFSUSB.onUnplug([](uint32_t) { rp2040.reboot(); });
    FatFSUSB.driveReady(usbDriveReady);
    FatFSUSB.begin();

    // Starting the mass-storage interface re-enumerates the USB device,
    // which drops the host's serial connection for about a second.
    // Everything printed in that window is lost - which used to include
    // the entire radio detection report. Wait for the host to come back
    // before continuing. Bounded, because on a charger there is no host.
    unsigned long usbWait = millis();
    while (!Serial && (millis() - usbWait) < 2500) delay(10);
    delay(250);

    green("USB mass storage active");

    // Print config summary
    snprintf(banner, sizeof(banner), " -- Tracker Booted: %s -=- %s", cfg.callsign, VERSION);
    red(banner);

    char cfgMsg[128];
    snprintf(cfgMsg, sizeof(cfgMsg), "SmartBeaconing: ON  Profile: %s", cfg.profile);
    yellow(cfgMsg);

    // ADC setup
    analogReadResolution(16);

    // EEPROM for sequence persistence
    EEPROM.begin(4);
    uint16_t stored = EEPROM.read(0) | (EEPROM.read(1) << 8);
    if (stored > 0 && stored <= 8191) {
        sequence = stored;
    }
    snprintf(cfgMsg, sizeof(cfgMsg), "Sequence start at: %d", sequence);
    yellow(cfgMsg);

    // LoRa init - the radio layer identifies the fitted chip itself
    yellow("Init LoRa");

    char radioErr[96] = "";
    if (!TrackerRadio::begin(cfg, radioErr, sizeof(radioErr))) {
        red(radioErr);
        red("LoRa INIT ERROR");
        while (true) { delay(1000); }
    }

    // The fitted radio is what the board revision actually means, and
    // it is read from the chip rather than inferred. The GP15 strap is
    // reported alongside it purely as an observation: it reads low on
    // V1 hardware as well, so it does not identify a revision on its
    // own and nothing functional depends on it.
    snprintf(cfgMsg, sizeof(cfgMsg), "Board: %s  (GP15 strap %s)",
             TrackerRadio::chip() == RADIO_CHIP_SX126X ? "V2" : "V1",
             boardV2 ? "low" : "open");
    yellow(cfgMsg);

    snprintf(cfgMsg, sizeof(cfgMsg), "Radio detected: %s", TrackerRadio::chipName());
    green(cfgMsg);
    if (TrackerRadio::chip() == RADIO_CHIP_SX126X) {
        float v = TrackerRadio::tcxoVoltage();
        if (v > 0.0f) snprintf(cfgMsg, sizeof(cfgMsg), "> clock: TCXO on DIO3 @ %.1fV", v);
        else          snprintf(cfgMsg, sizeof(cfgMsg), "> clock: crystal");
        yellow(cfgMsg);
        snprintf(cfgMsg, sizeof(cfgMsg), "> die regulator: %s, PA current limit %d mA",
                 TrackerRadio::usingLdoRegulator() ? "LDO" : "DC-DC", cfg.loraOcp);
        yellow(cfgMsg);
    }

    if (TrackerRadio::hasModulePa()) {
        snprintf(cfgMsg, sizeof(cfgMsg),
                 "LoRa OK (%.3f MHz, chip %d dBm into module PA, SF12/BW125/CR4:5)",
                 cfg.loraFrequency, TrackerRadio::appliedPower());
    } else {
        snprintf(cfgMsg, sizeof(cfgMsg),
                 "LoRa OK (%.3f MHz, pwr = %d dBm, SF12/BW125/CR4:5)",
                 cfg.loraFrequency, TrackerRadio::appliedPower());
    }
    green(cfgMsg);

    if (TrackerRadio::hasModulePa()) {
        // paDrive owns the output here; say so rather than letting a
        // stale power= line look effective.
        snprintf(cfgMsg, sizeof(cfgMsg),
                 "> paDrive=%d sets the drive; power=%d is unused on this module",
                 TrackerRadio::appliedPower(), cfg.power);
        yellow(cfgMsg);
    } else if (TrackerRadio::appliedPower() != cfg.power) {
        snprintf(cfgMsg, sizeof(cfgMsg),
                 "> power clamped from %d to %d dBm for this chip",
                 cfg.power, TrackerRadio::appliedPower());
        yellow(cfgMsg);
    }
    if (cfg.fullDebug) {
        char radioInfo[160];
        TrackerRadio::describe(radioInfo, sizeof(radioInfo));
        yellow(radioInfo);
    }

    // GPS init
    yellow("Init GPS");

    uint32_t baud = gpsBringUp(cfg);
    if (baud) {
        snprintf(cfgMsg, sizeof(cfgMsg), "GPS OK (%lu baud)", (unsigned long)baud);
        green(cfgMsg);
    } else {
        yellow("GPS silent at every baud rate tried - continuing without");
    }

    // I2C sensors
    if (cfg.i2cEnabled) {
        yellow("Init I2C sensors");
        digitalWrite(PIN_I2C_PWR, HIGH);
        delay(500);

        Wire1.setSDA(PIN_I2C_SDA);
        Wire1.setSCL(PIN_I2C_SCL);
        Wire1.setClock(100000);
        Wire1.begin();

        if (strcasecmp(cfg.i2cDevice, "BME680") == 0) {
            if (bme680.begin(0x77, &Wire1)) {
                hasBME680 = true;
                bme680.setTemperatureOversampling(BME680_OS_8X);
                bme680.setHumidityOversampling(BME680_OS_2X);
                bme680.setPressureOversampling(BME680_OS_4X);
                green("> BME680 OK");
            } else {
                yellow("> BME680 not found, continuing");
            }
        } else if (strcasecmp(cfg.i2cDevice, "SHTC3") == 0) {
            if (shtc3.begin(&Wire1)) {
                hasSHTC3 = true;
                green("> SHTC3 OK");
            } else {
                yellow("> SHTC3 not found, continuing");
            }
        }

        if (!hasBME680 && !hasSHTC3) {
            yellow("No sensors found, continuing without");
            digitalWrite(PIN_I2C_PWR, LOW);
        }
    }

    // Build and send telemetry metadata immediately (PA test - no GPS needed)
    buildMetadata();
    sendMetadata();

    // Init beacon engine
    sb.reset();
    jitter.reset();

    // Init GPS LED timing
    gpsLastBlink = millis() - (unsigned long)(cfg.gpsBlinkInterval * 1000);

    // NOW enable watchdog (5 sec) - setup is done
    watchdog_enable(5000, true);

    yellow("Start Tracking");
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
    bool hasFix = gps.location.isValid() && gps.location.age() < 3000;

    // Update GPS LED (always)
    if (!updateGpsLed(hasFix)) {
        delay(50);
        return;
    }

    // Need valid date/time
    if (!gps.date.isValid() || gps.date.year() < 2023) {
        if (cfg.fullDebug) {
            yellow("GPS timestamp not ready");
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
        char buf[200];
        snprintf(buf, sizeof(buf),
                 "FIX: LAT=%.6f LON=%.6f SPD=%.1fkm/h HDG=%.0f ST=%s JIT=%.0fm",
                 lat, lon,
                 speedKmh >= 0 ? speedKmh : 0,
                 heading >= 0 ? heading : 0,
                 jitter.isStationary() ? "Y" : "N",
                 movedM);
        purple(buf);
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
                char buf[64];
                snprintf(buf, sizeof(buf), "LOW VOLTAGE: %.2fV", vx100 / 100.0f);
                yellow(buf);
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
        char ts[8];
        aprsTimestamp(ts, 'z', gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second());

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

        char buf[350];
        snprintf(buf, sizeof(buf), "TX: %s", frame);
        purple(buf);
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
