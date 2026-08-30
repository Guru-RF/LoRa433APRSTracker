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
// Transmit drive used while a computer has the tracker enumerated
// over USB. Rated drive stalls the PA ramp on USB power; -9 is the
// SX1262's floor into a module amplifier and is still tens of dB
// above what a nearby receiver needs. See src/main.cpp.
#define DEFAULT_USB_PA_DRIVE  -9
// Hard ceiling for usbPaDrive, not a preference, and set by what is
// actually *received* rather than by what transmits without hanging.
// Measured against an iGate 8 m away: 8 decodes at -46 dBm, 7 at -43,
// 5 at -45, 0 at -50, -9 at -56 - RSSI tracking drive about 1:1. From
// 9 upward the tracker transmits cleanly, the console prints TX and
// the LED blinks, and nothing is ever decoded; 17 takes the whole USB
// link down and needs the cable physically pulled.
//
// So the dangerous band is 9..14: it looks like it is working and is
// not. That is the failure mode that cost an evening of debugging, and
// it is worth more than the 6 dB it would buy.
#define USB_PA_DRIVE_MAX      8
// Refuse to transmit at all while a computer is attached, rather than
// dropping to usbPaDrive. For a port that cannot supply even the
// floor drive.
#define DEFAULT_USB_TX_INHIBIT false
#define DEFAULT_LORA_FREQ     433.775f

// Radio chip: "auto" identifies SX1276/RFM95 vs SX1262 over SPI at boot.
// Override with sx1276 / sx1262 to pin it.
#define DEFAULT_RADIO_CHIP    "auto"
// SX126x clock source: "auto" asks the module, or give the TCXO supply
// voltage in volts ("0" for a plain crystal, "1.8", "3.3", ...).
#define DEFAULT_LORA_TCXO     "auto"
// Radio module type: "auto" uses the board strap, "minif27" for the
// G-NiceRF 1262MiniF27 with its internal PA, "bare" for a plain module.
#define DEFAULT_RADIO_MODULE  "auto"
// SX1262 output power in dBm driving a module's internal amplifier,
// -9..22. Only used on module-PA boards, where it replaces `power`
// entirely. Rated module output needs the die at its maximum.
#define DEFAULT_PA_DRIVE      22
// GPS UART baud. 0 = detect (V1 u-blox is 9600, V2 ATGM336H is 115200).
#define DEFAULT_GPS_BAUD      0
// SX1262 internal regulator: "dcdc" needs an inductor on the module's
// DCC_SW pin and halves the die's supply current; "ldo" always works.
// If the module has no inductor, dcdc starves the die's PA.
#define DEFAULT_LORA_REG      "dcdc"
// SX1262 PA over-current limit in mA, 0..157. Semtech's SX1262 default
// is 140; the die draws ~118 mA at +22 dBm, so there is little margin.
#define DEFAULT_LORA_OCP      140

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
    bool   usbTxInhibit;
    int    usbPaDrive;
    float  loraFrequency;
    char   radioChip[12];
    char   loraTcxo[8];
    char   radioModule[12];
    int    paDrive;
    long   gpsBaud;
    char   loraRegulator[8];
    int    loraOcp;

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
    cfg.usbTxInhibit = DEFAULT_USB_TX_INHIBIT;
    cfg.usbPaDrive = DEFAULT_USB_PA_DRIVE;
    cfg.loraFrequency = DEFAULT_LORA_FREQ;
    strncpy(cfg.radioChip, DEFAULT_RADIO_CHIP, sizeof(cfg.radioChip));
    strncpy(cfg.loraTcxo, DEFAULT_LORA_TCXO, sizeof(cfg.loraTcxo));
    strncpy(cfg.radioModule, DEFAULT_RADIO_MODULE, sizeof(cfg.radioModule));
    cfg.paDrive = DEFAULT_PA_DRIVE;
    cfg.gpsBaud = DEFAULT_GPS_BAUD;
    strncpy(cfg.loraRegulator, DEFAULT_LORA_REG, sizeof(cfg.loraRegulator));
    cfg.loraOcp = DEFAULT_LORA_OCP;

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
        cfg.power = constrain(atoi(val), -9, 23);
        // 18 and 19 are not producible on an SX1276 PA_BOOST; see radio.cpp
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
    } else if (strcasecmp(key, "usbTxInhibit") == 0) {
        cfg.usbTxInhibit = (strcasecmp(val, "true") == 0 || strcmp(val, "1") == 0);
    } else if (strcasecmp(key, "usbPaDrive") == 0) {
        cfg.usbPaDrive = constrain(atoi(val), -9, USB_PA_DRIVE_MAX);
    } else if (strcasecmp(key, "loraFrequency") == 0) {
        cfg.loraFrequency = atof(val);
    } else if (strcasecmp(key, "radioChip") == 0) {
        strncpy(cfg.radioChip, val, sizeof(cfg.radioChip) - 1);
    } else if (strcasecmp(key, "loraTcxo") == 0) {
        strncpy(cfg.loraTcxo, val, sizeof(cfg.loraTcxo) - 1);
    } else if (strcasecmp(key, "radioModule") == 0) {
        strncpy(cfg.radioModule, val, sizeof(cfg.radioModule) - 1);
    } else if (strcasecmp(key, "paDrive") == 0) {
        cfg.paDrive = constrain(atoi(val), -9, 22);
    } else if (strcasecmp(key, "gpsBaud") == 0) {
        cfg.gpsBaud = atol(val);
    } else if (strcasecmp(key, "loraRegulator") == 0) {
        strncpy(cfg.loraRegulator, val, sizeof(cfg.loraRegulator) - 1);
    } else if (strcasecmp(key, "loraOcp") == 0) {
        cfg.loraOcp = constrain(atoi(val), 0, 157);
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
    "# TX power in dBm (bare modules). Ignored on module-PA boards.\n"
    "power=23\n"
    "\n"
    "# Power amplifier present\n"
    "hasPa=true\n"
    "\n"
    "# LoRa frequency (MHz)\n"
    "loraFrequency=433.775\n"
    "\n"
    "# Radio chip: auto, sx1276 (RFM95) or sx1262\n"
    "radioChip=auto\n"
    "\n"
    "# SX1262 clock: auto, 0 (crystal), or TCXO volts e.g. 1.8 / 3.3\n"
    "loraTcxo=auto\n"
    "\n"
    "# Radio module: auto, minif27 (internal PA) or bare\n"
    "radioModule=auto\n"
    "\n"
    "# Chip dBm driving the module PA, -9..22 (22 = rated output)\n"
    "paDrive=22\n"
    "\n"
    "# GPS baud: 0 = detect, or pin it (9600 u-blox, 115200 ATGM336H)\n"
    "gpsBaud=0\n"
    "\n"
    "# SX1262 internal regulator: dcdc or ldo\n"
    "loraRegulator=dcdc\n"
    "# SX1262 PA current limit in mA, max 157\n"
    "loraOcp=140\n"
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
    "# --- USB ---\n"
    "# Drive used while a computer has the tracker enumerated, -9..8.\n"
    "# Higher transmits but is not decoded, so the firmware clamps it.\n"
    "usbPaDrive=-9\n"
    "# Or refuse to transmit at all while a computer is attached.\n"
    "usbTxInhibit=false\n"
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
    f.println("# Radio chip: auto, sx1276 (RFM95) or sx1262");
    f.println("radioChip=auto");
    f.println("# SX1262 clock: auto, 0 (crystal), or TCXO volts e.g. 1.8 / 3.3");
    f.println("loraTcxo=auto");
    f.println("# Radio module: auto, minif27 (internal PA) or bare");
    f.println("radioModule=auto");
    f.println("# Chip dBm driving the module PA, -9..22 (22 = rated output)");
    f.println("paDrive=22");
    f.println("# GPS baud: 0 = detect, or pin it (9600 u-blox, 115200 ATGM336H)");
    f.println("gpsBaud=0");
    f.println("# SX1262 internal regulator: dcdc or ldo");
    f.println("loraRegulator=dcdc");
    f.println("# SX1262 PA current limit in mA, max 157");
    f.println("loraOcp=140");
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
    f.println("# Drive used while a computer has the tracker enumerated, -9..8.");
    f.println("# Higher transmits but is not decoded, so the firmware clamps it.");
    f.println("usbPaDrive=-9");
    f.println("# Or refuse to transmit at all while a computer is attached.");
    f.println("usbTxInhibit=false");
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
