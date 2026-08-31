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
#include <tusb.h>

#include "pins.h"
#include "config_file.h"
#include "aprs.h"
#include "smartbeacon.h"
#include "radio.h"
#include "meshcore.h"

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

static unsigned long parkedSince = 0;
static bool          parkedSinceValid = false;
static unsigned long lastCommentSent = 0;
static bool          commentSent = false;
static unsigned long lastMeshAdvert = 0;
static bool          meshAdvertSent = false;

static const uint16_t SEQ_COMMIT_EVERY = 64;
static uint16_t sequence = 0;
static unsigned long lastDebugPrint = 0;
static unsigned long lastVoltageWarning = 0;
static unsigned long lastMetadataSend = 0;
static bool metadataForced = true;

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

// This maps the ADC onto 12.0..15.3 V, and it is right - validated against
// four days of production telemetry from a vehicle, which is the only
// evidence that counts here.
//
// It was briefly "corrected" to `ADC/0.6 + 10` on the reading that D5
// (PDZ10B) is a 10 V zener in series with the R18/R24 divider, making the
// front end a 10.0..15.5 V level shifter. That reading is wrong. Under it
// the same production data says the car charges at 12.7 V and sits at
// 10.1 V after four days parked - a battery that could not crank an
// engine, in a car that starts fine. Under the formula below the same
// samples read 13.6 V while driving and decay 12.46 -> 12.04 V over four
// days, which is a textbook lead-acid curve.
//
// Known limitation, and it matters for any battery-protection logic: the
// scale bottoms out at 12.0 V. A battery below that reads 12.0 and stops
// being distinguishable, so the useful decision band is roughly
// 12.0..13.8 V - enough to see "discharging" and "alternator running",
// not enough to see "flat". Characterising the front end properly needs a
// two-point bench measurement, not another look at the schematic.
static float getVoltage() {
    int pin = cfg.hasPa ? PIN_ADC_PA : PIN_ADC_NOPA;
    int raw = analogRead(pin);
    if (cfg.hasPa) {
        return ((raw * 3.3f) / 65536.0f) + 10.6f + 1.4f;
    }
    return ((raw * 3.3f) / 65536.0f) * 2.0f;
}

// ============================================================
// USB power budget
//
// This board cannot transmit at rated drive while a computer has it
// enumerated. The PA ramp does not complete, RadioLib waits on BUSY
// with no timeout of its own, and the tracker stops dead mid-frame -
// measured on a V2 board at paDrive 20 and 22, working at 14 and
// below, and working at 22 the moment USB-C is unplugged and it runs
// from the Powerpole alone.
//
// So the drive is chosen per frame instead: rated power when nothing
// is enumerated, cfg.usbPaDrive when something is. That follows the
// cable with no reboot, and unlike refusing to transmit it leaves the
// tracker audible on the bench - which is how you tell a working
// tracker from a silent one.
//
// The test is "is a host talking to us", not "is USB power present".
// tud_mounted() reports whether a host has completed SET_CONFIGURATION,
// which a charger and the 13.8 V Powerpole never do, so a deployed
// tracker always runs at full drive.
//
// Enumeration is the signal rather than the mass-storage mount because
// there is no dependable mount signal to use. FatFSUSB's onPlug only
// fires on a SCSI START STOP UNIT load, which no operating system
// sends when it mounts a volume, and PREVENT/ALLOW MEDIUM REMOVAL is
// optional per host. SET_CONFIGURATION is mandatory USB and behaves
// the same on macOS, Windows and Linux.
//
// usbTxInhibit=true keeps the old behaviour - silence rather than low
// power - for a port that cannot give even the floor drive.
// ============================================================

static const unsigned long USB_RELEASE_HOLD_MS  = 3000;
static const unsigned long USB_INHIBIT_LOG_MS   = 60000;
static const unsigned long USB_INHIBIT_BLINK_MS = 5000;

static bool          usbHost = false;       // debounced tud_mounted()
static bool          usbInhibit = false;
static unsigned long usbHostGoneSince = 0;
static bool          usbHostGoneSinceValid = false;
static unsigned long usbLastInhibitLog = 0;
static unsigned long usbBlinkStart = 0;

static bool usbHostPresent() {
    return usbHost;
}

static bool txInhibited() {
    return usbInhibit;
}

// ============================================================
// Battery protection
//
// Thresholds are measured, not assumed - four days of telemetry from a
// vehicle: alternator 13.62-13.66 V, settled 12.43-12.46 V, decaying
// ~0.105 V/day to 12.04 V. 13.00 V sits between parked-max and running,
// so it is an unambiguous "the engine is on"; 12.00 V is about 25% state
// of charge and is also the floor of the ADC scale.
//
// Two things make this safe to leave on by default.
//
// It arms itself. Protection does nothing until the tracker has once seen
// a charging voltage, which proves there is a battery and an alternator
// there at all. Without that, a tracker on USB or a bench supply reads the
// 12.0 V floor, looks exactly like a flat battery, and would throttle
// itself forever for no reason.
//
// It does not sample near a transmission. The PA pulls a few hundred
// milliamps and sags the rail, so a reading taken just after a frame would
// be of the sag rather than the battery.
// ============================================================

static const unsigned long BATT_SAMPLE_MS   = 30000;
static const unsigned long BATT_TX_QUIET_MS = 5000;

static bool          battArmed = false;      // a charging voltage has been seen
static bool          battLow = false;
static unsigned long battLastSample = 0;
static bool          battSampled = false;
static unsigned long lastTxEnd = 0;

static void battUpdate() {
    if (!cfg.battProtect || !cfg.voltage) { battLow = false; return; }

    unsigned long now = millis();
    if (battSampled && (now - battLastSample) < BATT_SAMPLE_MS) return;
    if (lastTxEnd && (now - lastTxEnd) < BATT_TX_QUIET_MS) return;   // rail still recovering
    battLastSample = now;
    battSampled = true;

    int vx100 = (int)(getVoltage() * 100 + 0.5f);
    bool was = battLow;

    if (vx100 >= cfg.battChargeVoltage) {
        if (!battArmed) {
            char buf[80];
            snprintf(buf, sizeof(buf), "Battery protection armed (charging at %.2fV)",
                     vx100 / 100.0f);
            green(buf);
        }
        battArmed = true;
        battLow = false;                       // alternator is running
    } else if (battArmed && vx100 <= cfg.battLowVoltage) {
        battLow = true;
    }

    if (battLow != was) {
        char buf[96];
        if (battLow) {
            snprintf(buf, sizeof(buf), "BATTERY LOW %.2fV - drive reduced to %d dBm",
                     vx100 / 100.0f, cfg.battPaDrive);
            red(buf);
        } else {
            snprintf(buf, sizeof(buf), "Battery recovered %.2fV - rated drive restored",
                     vx100 / 100.0f);
            green(buf);
        }
    }
}

static bool battProtecting() { return battLow; }

// The drive to transmit at right now, for either network. Both paths must
// use this: APRS setting it and MeshCore inheriting whatever was left over
// makes advert power depend on the order frames go out, and inheriting
// rated drive on USB puts the advert in the band that transmits and is
// never decoded.
static int8_t currentDrive() {
    int8_t rated = TrackerRadio::hasModulePa() ? (int8_t)cfg.paDrive
                                               : (int8_t)cfg.power;
    int8_t drive = usbHostPresent() ? (int8_t)cfg.usbPaDrive : rated;
    // A flat battery wins over anything else: never raise the drive here,
    // only lower it, so a USB clamp already in force still applies.
    if (battProtecting() && (int8_t)cfg.battPaDrive < drive) {
        drive = (int8_t)cfg.battPaDrive;
    }
    return drive;
}

// Recompute the host state. Called once per loop and once in setup(),
// before anything can decide to key the radio.
static void usbInhibitUpdate() {
    bool wasHost    = usbHost;
    bool wasInhibit = usbInhibit;

    if (tud_mounted()) {
        // Assert the moment a host appears. Being wrong the other way
        // costs a stalled transmitter, so there is nothing to debounce.
        usbHost = true;
        usbHostGoneSinceValid = false;
    } else if (usbHost) {
        // A bus reset zeroes the configuration for a moment. Hold until
        // the host has been gone long enough to mean it, rather than
        // stepping up to rated drive in the middle of a re-enumeration.
        if (!usbHostGoneSinceValid) {
            usbHostGoneSince = millis();
            usbHostGoneSinceValid = true;
        }
        if ((millis() - usbHostGoneSince) >= USB_RELEASE_HOLD_MS) {
            usbHost = false;
            usbHostGoneSinceValid = false;
        }
    }

    usbInhibit = cfg.usbTxInhibit && usbHost;

    (void)wasInhibit;
    if (usbHost && !wasHost) {
        char buf[96];
        snprintf(buf, sizeof(buf), "USB host connected - drive reduced to %d dBm",
                 cfg.usbPaDrive);
        yellow(cfg.usbTxInhibit ? "USB host connected - transmit inhibited" : buf);
        usbLastInhibitLog = millis();
        usbBlinkStart = millis();
    } else if (!usbHost && wasHost) {
        green("USB host disconnected - rated drive restored");
        digitalWrite(PIN_LED_PWR, HIGH);     // end any wink mid-cycle
    }
}

// Say what a host is doing to the transmitter, for as long as it does it.
//
// This covers BOTH states, and reduced drive is the dangerous one. A car
// head unit enumerates USB mass storage, so it holds the tracker at
// usbPaDrive - as little as -9 dBm - for a whole journey while the LoRa
// LED blinks normally and every frame is well formed. Nothing decodes
// them. That is the silent-transmit failure this firmware has already
// lost an evening to, and it deserves a standing indication rather than
// one line at boot. Nobody watches a serial
// console in a car, so the power LED - otherwise steady - winks
// briefly every 5 s alongside the console line.
//
// Deliberately NOT the LoRa LED. That one means "the transmitter is
// keyed", and blinking it to report the opposite reads as normal
// beaconing: it cost a bench session chasing an iGate that was
// receiving nothing because nothing was ever sent.
static void usbHostHold() {
    unsigned long now = millis();

    if ((now - usbLastInhibitLog) >= USB_INHIBIT_LOG_MS) {
        usbLastInhibitLog = now;
        if (usbInhibit) {
            yellow("TX inhibited - USB host still connected");
        } else {
            char buf[88];
            snprintf(buf, sizeof(buf),
                     "USB host still connected - drive held at %d dBm", cfg.usbPaDrive);
            yellow(buf);
        }
    }

    unsigned long phase = now - usbBlinkStart;
    if (phase >= USB_INHIBIT_BLINK_MS) {
        usbBlinkStart = now;
        phase = 0;
    }
    digitalWrite(PIN_LED_PWR, (phase < 100) ? LOW : HIGH);
}

// ============================================================
// LoRa Send
// ============================================================

static void loraSendText(const char *text) {
    // Backstop for the gate in loop(). Whatever reaches this function,
    // nothing is keyed while a host has the tracker enumerated - and
    // this sits above the PA enable below, so GP2 never rises either.
    if (txInhibited()) {
        yellow("TX inhibited - frame not sent");
        return;
    }

    watchdog_update();
    digitalWrite(PIN_LED_LORA, HIGH);

    // Rated drive stalls the PA ramp on USB power; see the USB power
    // budget notes above. Set per frame so unplugging the cable
    // restores full output on the very next beacon.
    TrackerRadio::setDrive(currentDrive());

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
    lastTxEnd = millis();
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
// MeshCore advert
//
// MeshCore stamps adverts with UNIX seconds and uses them for
// freshness, and this board has no RTC - so an advert waits for GPS
// time rather than inventing one. That means no mesh presence indoors,
// which is the honest trade: a mesh full of adverts dated 1970 is
// worse than a station that is quiet until it knows what time it is.
// ============================================================

static uint32_t gpsUnixTime() {
    if (!gps.date.isValid() || !gps.time.isValid() || gps.date.year() < 2023) return 0;

    // Days from the civil epoch (Howard Hinnant's algorithm), which is
    // exact and needs no table.
    int      y = gps.date.year();
    unsigned m = gps.date.month();
    unsigned d = gps.date.day();
    y -= m <= 2;
    const int      era = (y >= 0 ? y : y - 399) / 400;
    const unsigned yoe = (unsigned)(y - era * 400);
    const unsigned doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    const unsigned doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    const long     days = (long)era * 146097 + (long)doe - 719468;

    return (uint32_t)days * 86400UL + gps.time.hour() * 3600UL +
           gps.time.minute() * 60UL + gps.time.second();
}

static void sendMeshAdvert(float lat, float lon) {
    uint32_t now = gpsUnixTime();
    if (now == 0) return;

    digitalWrite(PIN_LED_LORA, HIGH);

    int8_t drive = currentDrive();
    char buf[128];
    snprintf(buf, sizeof(buf), "TX MESH: %s advert %s @ %.5f,%.5f (%d dBm)",
             cfg.meshNodeType, cfg.meshName, lat, lon, drive);
    purple(buf);

    if (!MeshCore::sendAdvert(cfg, lat, lon, now, drive)) {
        red("MESH ADVERT FAILED");
    }

    digitalWrite(PIN_LED_LORA, LOW);
    lastMeshAdvert = millis();
    meshAdvertSent = true;
    watchdog_update();
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
//
// ============================================================

static bool usbDriveReady(uint32_t) {
    return true;
}

// ============================================================
// SETUP
// ============================================================

void setup() {
    // Arm the watchdog before anything that can fail, not after.
    //
    // Three init paths end in `while (true)` - a failed FatFS mount, a
    // failed radio bring-up, and the post-reset_usb_boot spin. Unarmed,
    // each is a tracker that sits with a lit power LED and is dead until
    // someone physically repowers it. That is the same brick class the
    // watchdog was moved for once already; the reasoning was applied to
    // the metadata burst and not to the halts above it. A transient SPI
    // or oscillator fault on a cold winter start is exactly the kind of
    // thing that should reboot and retry rather than latch.
    //
    // 8 s here rather than the 5 s used in flight: the init path has
    // several fixed multi-second delays (GPS reset, USB re-enumeration,
    // the GPS baud sweep) and this leaves margin without needing a feed
    // inside each one. It is tightened to 5 s before the first frame.
    watchdog_enable(8000, true);

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

    // GPS reset (hold low 1s, then high 1s - same as Python boot.py)
    pinMode(PIN_GPS_RST, OUTPUT);
    digitalWrite(PIN_GPS_RST, LOW);
    delay(1000);
    watchdog_update();
    digitalWrite(PIN_GPS_RST, HIGH);
    delay(1000);
    watchdog_update();

    // USB Serial
    Serial.begin(115200);
    delay(2000);
    watchdog_update();

    Serial.print("\r\n");

    char banner[128];
    snprintf(banner, sizeof(banner), " -- Tracker Booted: %s -=- %s", cfg.callsign, VERSION);
    red(banner);

    // Init FAT filesystem and do all file ops BEFORE starting USB drive
    if (!FatFS.begin()) {
        red("FatFS init failed! (watchdog will reboot and retry)");
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

    // MeshCore identity, while the filesystem is still ours - once
    // FatFSUSB has it, the host owns it and we must not write.
    // The result is reported later: FatFSUSB.begin() below re-enumerates
    // the device and everything printed before the host comes back is
    // lost, which is exactly what hid this line the first time.
    char meshBoot[96] = "";
    bool meshBootOk = false;
    if (cfg.meshEnabled) {
        char meshErr[96] = "";
        if (MeshCore::begin(meshErr, sizeof(meshErr))) {
            const uint8_t *pk = MeshCore::publicKey();
            snprintf(meshBoot, sizeof(meshBoot),
                     "MeshCore identity: %s '%s' (%02X%02X%02X%02X...)",
                     cfg.meshNodeType, cfg.meshName, pk[0], pk[1], pk[2], pk[3]);
            meshBootOk = true;
        } else {
            snprintf(meshBoot, sizeof(meshBoot), "%s", meshErr);
            cfg.meshEnabled = false;
        }
    }

    // Start USB mass storage (after all file operations). Ejecting
    // reboots so the new config takes effect - and nothing more, now
    // that the transmitter no longer depends on the eject.
    FatFSUSB.onUnplug([](uint32_t) { rp2040.reboot(); });
    FatFSUSB.driveReady(usbDriveReady);
    FatFSUSB.begin();
    watchdog_update();

    // Starting the mass-storage interface re-enumerates the USB device,
    // which drops the host's serial connection for about a second.
    // Everything printed in that window is lost - which used to include
    // the entire radio detection report. Wait for the host to come back
    // before continuing. Bounded, because on a charger there is no host.
    unsigned long usbWait = millis();
    while (!Serial && (millis() - usbWait) < 2500) { delay(10); watchdog_update(); }
    delay(250);
    watchdog_update();

    green("USB mass storage active");

    if (meshBoot[0]) {
        if (meshBootOk) {
            green(meshBoot);
        } else {
            red(meshBoot);
            yellow("MeshCore disabled for this boot");
        }
    }

    // Print config summary
    snprintf(banner, sizeof(banner), " -- Tracker Booted: %s -=- %s", cfg.callsign, VERSION);
    red(banner);

    char cfgMsg[128];
    snprintf(cfgMsg, sizeof(cfgMsg), "SmartBeaconing: ON  Profile: %s", cfg.profile);
    yellow(cfgMsg);

    // ADC setup
    analogReadResolution(16);

    // EEPROM for sequence persistence
    // The sequence counter is only persisted every SEQ_COMMIT_EVERY
    // beacons - see the beacon block - so the stored value can be up to
    // that far behind. Resume a whole block past it rather than replaying
    // numbers already on the air.
    EEPROM.begin(4);
    uint16_t stored = EEPROM.read(0) | (EEPROM.read(1) << 8);
    if (stored > 0 && stored <= 8191) {
        sequence = (uint16_t)((stored + SEQ_COMMIT_EVERY) % 8192);
    }
    snprintf(cfgMsg, sizeof(cfgMsg), "Sequence start at: %d", sequence);
    yellow(cfgMsg);

    // LoRa init - the radio layer identifies the fitted chip itself
    yellow("Init LoRa");

    char radioErr[96] = "";
    if (!TrackerRadio::begin(cfg, radioErr, sizeof(radioErr))) {
        red(radioErr);
        red("LoRa INIT ERROR (watchdog will reboot and retry)");
        while (true) { delay(1000); }
    }

    // The fitted radio is what the board revision means, and it is read
    // off the chip rather than inferred from any strap.
    snprintf(cfgMsg, sizeof(cfgMsg), "Board: %s",
             TrackerRadio::chip() == RADIO_CHIP_SX126X ? "V2" : "V1");
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
    if (cfg.meshEnabled) {
        snprintf(cfgMsg, sizeof(cfgMsg),
                 "MeshCore: %.3f MHz BW%.1f SF%d CR4:%d, %s advert every %ds",
                 cfg.meshFrequency, cfg.meshBandwidth, cfg.meshSf, cfg.meshCr,
                 cfg.meshRoute, cfg.meshInterval);
        yellow(cfgMsg);
    }

    if (cfg.fullDebug && cfg.voltage) {
        // Raw counts alongside the volts, so one boot with a meter on the
        // Powerpole confirms or corrects ADC_ZENER_V / ADC_DIVIDER instead
        // of anyone having to infer them from a beacon's telemetry.
        int rawAdc = analogRead(cfg.hasPa ? PIN_ADC_PA : PIN_ADC_NOPA);
        snprintf(cfgMsg, sizeof(cfgMsg), "ADC raw=%d -> %.2f V", rawAdc, getVoltage());
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
        watchdog_update();

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
    //
    // The call is skipped rather than the transmission inside it:
    // sendMetadata() clears metadataForced as its first act, so
    // suppressing the RF instead would leave receivers with no
    // PARM/UNIT/EQNS to decode telemetry with for the next 24 hours.
    buildMetadata();

    // Arm the watchdog *before* the first transmission, not after setup.
    //
    // RadioLib waits for BUSY to fall after SetTx with no timeout of its
    // own, so a module whose PA cannot ramp spins there forever. That is
    // survivable in loop(), where the watchdog resets the board - but
    // these three metadata frames used to go out while the watchdog was
    // still off, so the same stall bricked the tracker until someone
    // physically unplugged it. Observed on a V2 board: power LED lit,
    // LoRa LED stuck on mid-frame, USB gone, dead until repowered.
    //
    // Everything from here on either feeds the watchdog or completes in
    // well under 5 s, so there is nothing left that needs it disabled.
    watchdog_enable(5000, true);

    usbInhibitUpdate();
    if (txInhibited()) {
        yellow("Telemetry metadata held while a USB host is connected");
    } else {
        sendMetadata();
    }

    // Init beacon engine
    sb.reset();
    jitter.reset();

    // Init GPS LED timing
    gpsLastBlink = millis() - (unsigned long)(cfg.gpsBlinkInterval * 1000);

    yellow("Start Tracking");
}

// ============================================================
// MAIN LOOP
// ============================================================

void loop() {
    watchdog_update();

    // Recompute the inhibit before anything can decide to key the
    // radio. GPS, the LEDs and the console keep running while it
    // holds - only the transmitter stops.
    battUpdate();
    usbInhibitUpdate();
    if (usbHostPresent()) {
        usbHostHold();
    }
    if (!txInhibited() && metadataForced) {
        // Held back at boot because a host was connected. Send it as
        // soon as the inhibit lifts, without waiting for a GPS fix, so
        // a bench PA test still gets its metadata.
        sendMetadata();
    }

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

    // Everything below this line does nothing but decide to key the
    // transmitter, so the inhibit stops here rather than at the radio.
    // Taking the decision and then suppressing the RF would burn a
    // sequence number into flash for a frame nobody hears, stamp
    // lastVoltageWarning on an alert that never went out, and clear
    // metadataForced. It has to stay above sb.shouldBeacon() in
    // particular: suppressing between that call and updateAfterBeacon()
    // leaves the SmartBeacon anchor unset, which makes shouldBeacon()
    // true on every 50 ms pass and erases a flash sector each time.
    //
    // jitter.stabilize() above keeps running on purpose - it is the
    // position filter and its anchor needs continuous GPS.
    if (txInhibited()) {
        delay(50);
        return;
    }

    // A tracker left parked does not need a position every three minutes,
    // and the car's battery is the thing paying for it. After sbParkedAfter
    // of continuous stillness the stationary rate drops to sbParkedRate.
    //
    // Worth about 10% of the average draw, not more: transmitting is only
    // ~11% of it, and the continuous GPS and MCU load dominates. Measured
    // decay on a real vehicle is ~0.105 V/day, so this trims rather than
    // solves that.
    if (jitter.isStationary()) {
        if (!parkedSinceValid) { parkedSince = nowMs; parkedSinceValid = true; }
    } else {
        parkedSinceValid = false;
    }
    bool longParked = cfg.sbParkedAfter > 0 && parkedSinceValid &&
                      (nowMs - parkedSince) >= (unsigned long)cfg.sbParkedAfter * 1000UL;
    int slowRate = longParked ? cfg.sbParkedRate : cfg.sbSlowRate;

    // Determine whether to beacon
    float sbSpeed = jitter.isStationary() ? 0.0f : (speedKmh >= 0 ? speedKmh : 0);
    bool sendBeacon = startedMoving || sb.shouldBeacon(
        lat, lon, sbSpeed, heading,
        cfg.sbStationarySpeed, cfg.sbSlowSpeed, cfg.sbFastSpeed,
        slowRate, cfg.sbFastRate,
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

    bool meshDue = cfg.meshEnabled && MeshCore::ready() &&
                   (!meshAdvertSent ||
                    (nowMs - lastMeshAdvert) >= (unsigned long)cfg.meshInterval * 1000UL);

    // Nothing to do?
    if (!sendBeacon && pendingVoltAlert < 0 && !metadataDue && !meshDue) {
        delay(50);
        return;
    }

    // MeshCore advert, on a pass of its own.
    //
    // Never in the same pass as an APRS beacon, metadata or an alert: the
    // two networks use different on-air profiles, so a retune must not land
    // between the frames of something else. Yielding costs one 50 ms pass.
    if (meshDue && !sendBeacon && pendingVoltAlert < 0 && !metadataDue) {
        sendMeshAdvert(lat, lon);
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

        // Altitude rides in the compressed position's cs bytes while
        // parked - three bytes the report was wasting, against nine for a
        // /A= in the comment. Moving, course and speed earn the field and
        // /A= carries the altitude instead.
        //
        // The test is speed, not whether a course happens to be
        // representable. A parked GPS still reports a heading, so keying
        // on that encoded "course 10 degrees, speed 0" into cs and then
        // paid for /A= anyway - misfiring in exactly the case the
        // optimisation exists for.
        char aprsPos[20];
        float spdForAprs = speedKmh >= 0 ? speedKmh : -1;
        float hdgForAprs = heading >= 0 ? heading : -1;
        bool  parked     = jitter.isStationary() ||
                           (spdForAprs >= 0 && spdForAprs < cfg.sbStationarySpeed);
        float altFt      = (altM >= 0 && cfg.aprsAltitude) ? altM * 3.2808399f : -1.0f;
        aprsPosition(aprsPos, lat, lon,
                     parked ? -1.0f : spdForAprs,
                     parked ? -1.0f : hdgForAprs,
                     cfg.symbol,
                     parked ? altFt : -1.0f);

        // Sequence
        // EEPROM.commit() erases a 4 KB flash sector, and doing that on
        // every beacon is the heaviest wear in the firmware: driving at
        // fastRate that is ~1440 erases a day, reaching the 100k-cycle
        // datasheet minimum in about ten weeks. All that is being kept is
        // an APRS telemetry sequence number whose only cost on loss is
        // that telemetry restarts, so it is persisted a block at a time
        // and resumed a block ahead - 64x the endurance, same behaviour
        // on the air.
        sequence = (sequence + 1) % 8192;
        if ((sequence % SEQ_COMMIT_EVERY) == 0) {
            EEPROM.write(0, sequence & 0xFF);
            EEPROM.write(1, (sequence >> 8) & 0xFF);
            EEPROM.commit();
        }

        // Build comment with telemetry
        // aprs.fi keeps the last comment it saw and only drops it after
        // seven days of comment-less packets, so sending it on every beacon
        // buys nothing - and it is a third of the frame. Timed rather than
        // counted, so the gap does not shrink to nothing at speed. The
        // first beacon of a session always carries it.
        bool withComment = (cfg.commentInterval <= 0) || !commentSent ||
                           (nowMs - lastCommentSent) >=
                               (unsigned long)cfg.commentInterval * 1000UL;
        if (withComment) {
            lastCommentSent = nowMs;
            commentSent = true;
        }

        char comment[128];
        int cpos = 0;
        cpos += snprintf(comment + cpos, sizeof(comment) - cpos, "%s|",
                         withComment ? cfg.comment : "");

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

        // Only while moving - parked, the cs field already carries it.
        if (altM >= 0 && cfg.aprsAltitude && !parked) {
            int feet = (int)(altM * 3.2808399f);
            cpos += snprintf(comment + cpos, sizeof(comment) - cpos, "/A=%06d", feet);
        }

        // Build full APRS frame
        char frame[300];
        // '@' carries the timestamp and claims message capability; '!' is
        // real-time with neither. This tracker cannot receive, so '!' is the
        // honest form as well as the shorter one - the same choice the
        // RF.Guru iGate makes - and it saves 8 bytes, about 10% of the air
        // time, because receivers stamp on arrival anyway.
        // '/' is timestamp-without-messaging; '@' is timestamp-with. This
        // tracker has no receive path at all, so '@' advertises a
        // capability it does not have - the same reason the untimestamped
        // branch uses '!' rather than '='.
        if (cfg.aprsTimestamp) {
            snprintf(frame, sizeof(frame), "%s>APRFGT:/%s%s%s",
                     cfg.callsign, ts, aprsPos, comment);
        } else {
            snprintf(frame, sizeof(frame), "%s>APRFGT:!%s%s",
                     cfg.callsign, aprsPos, comment);
        }

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
