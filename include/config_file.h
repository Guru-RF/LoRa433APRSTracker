// ============================================================
// RF.Guru LoRa 433 APRS Tracker - Config File Reader
// Reads key=value config from FAT filesystem
// ============================================================

#pragma once

#include <Arduino.h>
#include <FatFS.h>

extern FS FatFS;

// Config defaults
#define DEFAULT_CALLSIGN      "ON9RFG"
#define DEFAULT_SYMBOL        "L>"
#define DEFAULT_COMMENT       "https://RF.Guru"
#define DEFAULT_PROFILE       "car"
#define DEFAULT_POWER         23
#define DEFAULT_HAS_PA        true
#define DEFAULT_VOLTAGE       true
#define DEFAULT_TRIGGER_VOLT  true
#define DEFAULT_TRIGGER_LEVEL 1200
#define DEFAULT_TRIGGER_CALL  "ON9RFG"
#define DEFAULT_TRIGGER_KEEP  3600
#define DEFAULT_I2C_ENABLED   true
#define DEFAULT_I2C_DEVICE    "BME680"
#define DEFAULT_BME680_OFFSET 0
#define DEFAULT_FULL_DEBUG    false
#define DEFAULT_LORA_FREQ     433.775f

// GPS LED defaults
#define DEFAULT_GPS_BLINK_INTERVAL  2.0f
#define DEFAULT_GPS_BLINK_PULSE     0.10f
#define DEFAULT_GPS_NOFIX_LOG_INT   5.0f
#define DEFAULT_GPS_LOCK_HOLD       0.0f
#define DEFAULT_GPS_UNLOCK_HOLD     2.0f

// SmartBeacon presets
struct SBPreset {
    int    fastRate;
    int    slowRate;
    int    fastSpeed;
    int    slowSpeed;
    float  stationarySpeed;
    int    stationaryDistance;
    int    stationaryExitCount;
    int    turnThreshold;
    float  turnSlope;
    bool   headingFilter;
};

static const SBPreset PRESET_CAR = {
    15, 180, 60, 10, 1.0f, 100, 3, 30, 5.0f, true
};
static const SBPreset PRESET_BIKE = {
    25, 240, 25, 5, 0.6f, 100, 3, 22, 4.0f, true
};
static const SBPreset PRESET_HIKER = {
    40, 360, 10, 1, 0.4f, 100, 3, 18, 3.0f, true
};

struct TrackerConfig {
    char   callsign[16];
    char   symbol[4];
    char   comment[64];
    char   profile[16];
    int    power;
    bool   hasPa;
    bool   voltage;
    bool   triggerVoltage;
    int    triggerVoltageLevel;
    char   triggerVoltageCall[16];
    int    triggerVoltageKeepalive;
    bool   i2cEnabled;
    char   i2cDevice[16];
    int    bme680TempOffset;
    bool   fullDebug;
    float  loraFrequency;

    // GPS LED
    float  gpsBlinkInterval;
    float  gpsBlinkPulse;
    float  gpsNoFixLogInterval;
    float  gpsLockHold;
    float  gpsUnlockHold;

    // SmartBeacon (resolved from profile)
    int    sbFastRate;
    int    sbSlowRate;
    int    sbFastSpeed;
    int    sbSlowSpeed;
    float  sbStationarySpeed;
    int    sbStationaryDistance;
    int    sbStationaryExitCount;
    int    sbTurnThreshold;
    float  sbTurnSlope;
    bool   sbHeadingFilter;
};

static void configSetDefaults(TrackerConfig &cfg) {
    strncpy(cfg.callsign, DEFAULT_CALLSIGN, sizeof(cfg.callsign));
    strncpy(cfg.symbol, DEFAULT_SYMBOL, sizeof(cfg.symbol));
    strncpy(cfg.comment, DEFAULT_COMMENT, sizeof(cfg.comment));
    strncpy(cfg.profile, DEFAULT_PROFILE, sizeof(cfg.profile));
    cfg.power = DEFAULT_POWER;
    cfg.hasPa = DEFAULT_HAS_PA;
    cfg.voltage = DEFAULT_VOLTAGE;
    cfg.triggerVoltage = DEFAULT_TRIGGER_VOLT;
    cfg.triggerVoltageLevel = DEFAULT_TRIGGER_LEVEL;
    strncpy(cfg.triggerVoltageCall, DEFAULT_TRIGGER_CALL, sizeof(cfg.triggerVoltageCall));
    cfg.triggerVoltageKeepalive = DEFAULT_TRIGGER_KEEP;
    cfg.i2cEnabled = DEFAULT_I2C_ENABLED;
    strncpy(cfg.i2cDevice, DEFAULT_I2C_DEVICE, sizeof(cfg.i2cDevice));
    cfg.bme680TempOffset = DEFAULT_BME680_OFFSET;
    cfg.fullDebug = DEFAULT_FULL_DEBUG;
    cfg.loraFrequency = DEFAULT_LORA_FREQ;

    cfg.gpsBlinkInterval = DEFAULT_GPS_BLINK_INTERVAL;
    cfg.gpsBlinkPulse = DEFAULT_GPS_BLINK_PULSE;
    cfg.gpsNoFixLogInterval = DEFAULT_GPS_NOFIX_LOG_INT;
    cfg.gpsLockHold = DEFAULT_GPS_LOCK_HOLD;
    cfg.gpsUnlockHold = DEFAULT_GPS_UNLOCK_HOLD;
}

static void configApplyProfile(TrackerConfig &cfg) {
    const SBPreset *p = &PRESET_CAR;
    if (strcasecmp(cfg.profile, "bike") == 0) {
        p = &PRESET_BIKE;
    } else if (strcasecmp(cfg.profile, "hiker") == 0) {
        p = &PRESET_HIKER;
    }
    cfg.sbFastRate          = p->fastRate;
    cfg.sbSlowRate          = p->slowRate;
    cfg.sbFastSpeed         = p->fastSpeed;
    cfg.sbSlowSpeed         = p->slowSpeed;
    cfg.sbStationarySpeed   = p->stationarySpeed;
    cfg.sbStationaryDistance = p->stationaryDistance;
    cfg.sbStationaryExitCount = p->stationaryExitCount;
    cfg.sbTurnThreshold     = p->turnThreshold;
    cfg.sbTurnSlope         = p->turnSlope;
    cfg.sbHeadingFilter     = p->headingFilter;
}

static void trimWhitespace(char *s) {
    // Trim leading
    char *start = s;
    while (*start == ' ' || *start == '\t') start++;
    if (start != s) memmove(s, start, strlen(start) + 1);
    // Trim trailing
    int len = strlen(s);
    while (len > 0 && (s[len - 1] == ' ' || s[len - 1] == '\t' ||
                        s[len - 1] == '\r' || s[len - 1] == '\n')) {
        s[--len] = '\0';
    }
}

static void configSetValue(TrackerConfig &cfg, const char *key, const char *val) {
    if (strcasecmp(key, "callsign") == 0) {
        strncpy(cfg.callsign, val, sizeof(cfg.callsign) - 1);
        // Uppercase callsign
        for (char *p = cfg.callsign; *p; p++) *p = toupper(*p);
    } else if (strcasecmp(key, "symbol") == 0) {
        strncpy(cfg.symbol, val, sizeof(cfg.symbol) - 1);
    } else if (strcasecmp(key, "comment") == 0) {
        strncpy(cfg.comment, val, sizeof(cfg.comment) - 1);
    } else if (strcasecmp(key, "profile") == 0) {
        strncpy(cfg.profile, val, sizeof(cfg.profile) - 1);
    } else if (strcasecmp(key, "power") == 0) {
        cfg.power = constrain(atoi(val), 5, 23);
    } else if (strcasecmp(key, "hasPa") == 0) {
        cfg.hasPa = (strcasecmp(val, "true") == 0 || strcmp(val, "1") == 0);
    } else if (strcasecmp(key, "voltage") == 0) {
        cfg.voltage = (strcasecmp(val, "true") == 0 || strcmp(val, "1") == 0);
    } else if (strcasecmp(key, "triggerVoltage") == 0) {
        cfg.triggerVoltage = (strcasecmp(val, "true") == 0 || strcmp(val, "1") == 0);
    } else if (strcasecmp(key, "triggerVoltageLevel") == 0) {
        cfg.triggerVoltageLevel = constrain(atoi(val), 250, 1280);
    } else if (strcasecmp(key, "triggerVoltageCall") == 0) {
        strncpy(cfg.triggerVoltageCall, val, sizeof(cfg.triggerVoltageCall) - 1);
        for (char *p = cfg.triggerVoltageCall; *p; p++) *p = toupper(*p);
    } else if (strcasecmp(key, "triggerVoltageKeepalive") == 0) {
        cfg.triggerVoltageKeepalive = constrain(atoi(val), 3600, 14400);
    } else if (strcasecmp(key, "i2cEnabled") == 0) {
        cfg.i2cEnabled = (strcasecmp(val, "true") == 0 || strcmp(val, "1") == 0);
    } else if (strcasecmp(key, "i2cDevice") == 0) {
        strncpy(cfg.i2cDevice, val, sizeof(cfg.i2cDevice) - 1);
    } else if (strcasecmp(key, "bme680TempOffset") == 0) {
        cfg.bme680TempOffset = constrain(atoi(val), 0, 99);
    } else if (strcasecmp(key, "fullDebug") == 0) {
        cfg.fullDebug = (strcasecmp(val, "true") == 0 || strcmp(val, "1") == 0);
    } else if (strcasecmp(key, "loraFrequency") == 0) {
        cfg.loraFrequency = atof(val);
    } else if (strcasecmp(key, "gpsBlinkInterval") == 0) {
        cfg.gpsBlinkInterval = atof(val);
    } else if (strcasecmp(key, "gpsBlinkPulse") == 0) {
        cfg.gpsBlinkPulse = atof(val);
    } else if (strcasecmp(key, "gpsNoFixLogInterval") == 0) {
        cfg.gpsNoFixLogInterval = atof(val);
    } else if (strcasecmp(key, "gpsLockHold") == 0) {
        cfg.gpsLockHold = atof(val);
    } else if (strcasecmp(key, "gpsUnlockHold") == 0) {
        cfg.gpsUnlockHold = atof(val);
    }
}

static const char *CONFIG_TEMPLATE =
    "# ============================================================\n"
    "# RF.Guru LoRa 433 APRS Tracker Configuration\n"
    "# Edit this file and reboot the device to apply changes\n"
    "# ============================================================\n"
    "\n"
    "# Profile: car, bike, or hiker (sets SmartBeacon parameters)\n"
    "profile=car\n"
    "\n"
    "# Callsign (APRS)\n"
    "callsign=ON9RFG\n"
    "\n"
    "# APRS symbol (2 chars: table + code, e.g. L> = LoRa car)\n"
    "symbol=L>\n"
    "\n"
    "# APRS comment appended to position reports\n"
    "comment=https://RF.Guru\n"
    "\n"
    "# TX power (5-23 dBm)\n"
    "power=23\n"
    "\n"
    "# Power amplifier present\n"
    "hasPa=true\n"
    "\n"
    "# LoRa frequency (MHz)\n"
    "loraFrequency=433.775\n"
    "\n"
    "# --- Voltage Monitoring ---\n"
    "voltage=true\n"
    "triggerVoltage=true\n"
    "triggerVoltageLevel=1200\n"
    "triggerVoltageCall=ON9RFG\n"
    "triggerVoltageKeepalive=3600\n"
    "\n"
    "# --- I2C Sensors ---\n"
    "i2cEnabled=true\n"
    "i2cDevice=BME680\n"
    "bme680TempOffset=0\n"
    "\n"
    "# --- Debug ---\n"
    "fullDebug=false\n";

static bool configCreateDefault() {
    File f = FatFS.open("/config.txt", "w");
    if (!f) {
        Serial.println("[CONFIG] ERROR: Could not create file");
        return false;
    }
    // Write line by line to avoid large buffer issues
    f.println("# RF.Guru LoRa 433 APRS Tracker Configuration");
    f.println("# Edit this file and reboot/eject the device to apply");
    f.println("");
    f.println("profile=car");
    f.println("callsign=ON9RFG");
    f.println("symbol=L>");
    f.println("comment=https://RF.Guru");
    f.println("power=23");
    f.println("hasPa=true");
    f.println("loraFrequency=433.775");
    f.println("");
    f.println("voltage=true");
    f.println("triggerVoltage=true");
    f.println("triggerVoltageLevel=1200");
    f.println("triggerVoltageCall=ON9RFG");
    f.println("triggerVoltageKeepalive=3600");
    f.println("");
    f.println("i2cEnabled=true");
    f.println("i2cDevice=BME680");
    f.println("bme680TempOffset=0");
    f.println("");
    f.println("fullDebug=false");
    f.close();
    Serial.println("[CONFIG] Default config.txt created");
    return true;
}

static bool configLoad(TrackerConfig &cfg) {
    configSetDefaults(cfg);

    if (!FatFS.exists("/config.txt")) {
        Serial.println("[CONFIG] No config.txt found, creating default...");
        configCreateDefault();
    }

    File f = FatFS.open("/config.txt", "r");
    if (!f) {
        Serial.println("[CONFIG] ERROR: Could not read config.txt");
        return false;
    }

    char line[128];
    while (f.available()) {
        int len = 0;
        char c;
        memset(line, 0, sizeof(line));
        while (f.available() && len < (int)sizeof(line) - 1) {
            c = f.read();
            if (c == '\n') break;
            line[len++] = c;
        }
        line[len] = '\0';
        trimWhitespace(line);

        // Skip empty lines and comments
        if (line[0] == '\0' || line[0] == '#') continue;

        // Find '='
        char *eq = strchr(line, '=');
        if (!eq) continue;

        *eq = '\0';
        char *key = line;
        char *val = eq + 1;
        trimWhitespace(key);
        trimWhitespace(val);

        configSetValue(cfg, key, val);
    }

    f.close();

    configApplyProfile(cfg);

    Serial.printf("[CONFIG] Loaded: callsign=%s profile=%s power=%d freq=%.3f\n",
                  cfg.callsign, cfg.profile, cfg.power, cfg.loraFrequency);
    return true;
}
