// ============================================================
// RF.Guru LoRa 433 APRS Tracker - Config File Reader
// Reads key=value config from FAT filesystem
// ============================================================

#pragma once

#include <Arduino.h>
#include <FatFS.h>
#include <hardware/watchdog.h>

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

// Boot-burst diagnostics. Something loses the EARLIEST frames of the
// three-frame metadata burst on OR7F-5 whenever the module PA is actually
// amplifying, and only the last frame sent survives. Two explanations
// predict opposite outcomes: an iGate that drops a frame when the next
// preamble arrives 150 ms behind it wants a longer gap; a PA that is dirty
// until it has run for a while wants a warm-up before the first frame.
// Both default to the behaviour we always had.
#define DEFAULT_META_GAP      150     // ms between the burst's frames
#define DEFAULT_PA_WARMUP     0       // ms of unmodulated carrier before it
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

// --- MeshCore (IARU R1 ham profile) ---
// Off until the operator provisions it: enabling mints an Ed25519
// identity and puts the station on a shared network under its own name.
// Frame content. Both default to what the firmware has always sent, so a
// firmware update does not silently change what deployed trackers put on
// the air - turning either off is the operator's decision.
//
// At SF12 the payload quantises in 5-byte steps of 164 ms, so trimming
// fewer than 5 bytes buys nothing. Each of these is worth about 10%:
// the timestamp is 8 bytes and receivers stamp on arrival anyway, and
// altitude is 9.
// --- Battery protection ---
// Thresholds measured on a production vehicle over four days, not taken
// from a textbook: the alternator holds 13.62-13.66 V, the battery settles
// at 12.43-12.46 V once parked, and decays ~0.105 V/day to 12.04 V after
// four days. 13.00 sits cleanly between parked-max and running; 12.00 is
// about 25% state of charge and is also where the ADC scale bottoms out,
// so nothing below it is measurable - which is the point. Act before the
// battery is dead, not after.
#define DEFAULT_BATT_PROTECT     true
#define DEFAULT_BATT_LOW_V       1200   // V*100 - below this, back off
#define DEFAULT_BATT_CHARGE_V    1300   // V*100 - above this, alternator is on
#define DEFAULT_BATT_PA_DRIVE    10     // drive while the battery is unsupported
// Seconds after boot before battery protection may act. Only useful on a
// tracker wired to switched/ignition power, where it boots as the engine
// cranks and the first reading is of the starter motor rather than the
// battery. On permanent power - which is how a tracker that beacons all
// night is wired - cranking happens mid-run and this covers nothing; the
// consecutive-sample rule in src/main.cpp is what handles that case.
// 0 disables it, which is the right setting for an EV: no starter motor,
// no crank dip, and the 12 V DC-DC comes up immediately.
#define DEFAULT_BATT_START_DELAY 10
// A rail that is moving is a rail not to key a transmitter into. Cranking
// is a dip AND a rise - the starter pulls the battery down, then the
// alternator comes up - and both edges are worth staying off the air for.
// Detected as a step between one-second samples, in hundredths of a volt,
// which is why this is separate from the level thresholds above: a level
// says how full the battery is, a step says the rail is not settled.
// 0 disables it.
#define DEFAULT_BATT_STEP_V      30     // V*100 between samples = unstable
#define DEFAULT_BATT_SETTLE      5      // seconds off the air afterwards

// --- Parked ---
// Stationary this long and the beacon rate drops to sbParkedRate. 0 = off.
// Worth roughly 10% of the tracker's average draw; the continuous GPS and
// MCU load dominates, so this is a modest saving rather than a large one.
// --- Alerts ---
// A tracker in a vehicle has no console, so the diagnostics that matter -
// the supply moving, the battery falling, protection arming - are queued
// and sent on the air instead. Both destinations are optional and both are
// empty by default: this costs airtime, and a shared channel is not the
// place for chatter nobody asked for.
#define DEFAULT_ALERT_CHANNEL  ""     // MeshCore public channel, e.g. #mbox
#define DEFAULT_ALERT_MENTION  ""     // prefixed to the text, e.g. @OR7F
#define DEFAULT_ALERT_APRS     ""     // APRS callsign to message, e.g. OR7F-1
#define DEFAULT_ALERT_MESHCOM  ""     // MeshCom callsign to PM, e.g. OR7F-90
#define DEFAULT_ALERT_INTERVAL 60     // minimum seconds between alerts

#define DEFAULT_SB_PARKED_AFTER  14400  // 4 hours
#define DEFAULT_SB_PARKED_RATE   1800   // seconds between beacons once parked

#define DEFAULT_APRS_TIMESTAMP true
#define DEFAULT_APRS_ALTITUDE  true
// Seconds between comments, not a beacon count. A count means different
// things at different speeds - every 3rd beacon is 9 minutes parked but
// 3 minutes on the motorway, which is backwards, since airtime is
// scarcest exactly when the tracker is moving fastest.
//
// aprs.fi keeps the last comment it saw and, per its author, only
// "forgets it if you still transmit packets without a comment after 7
// days", so 1800 s has a 336x margin. The comment is a third of the
// frame and this recovers ~30% of the airtime; an hour instead of half
// buys a further 2%, so there is no reason to push it.
//
// "always" (or 0) sends it on every beacon, for a network whose consumers
// do not cache it the way aprs.fi does. "off" drops it from the APRS
// beacon entirely - for a station whose free text lives on the MeshCom
// side, where the position carries a prefixed copy of the same comment
// and the network has no aprs.fi to cache it.
#define DEFAULT_COMMENT_INTERVAL 1800
#define COMMENT_INTERVAL_OFF     (-1)

#define DEFAULT_MESH_ENABLED   false
#define DEFAULT_MESH_NAME      "NOCALL"
// chat | repeater | room | sensor. A repeater gates chat and repeater
// adverts to APRS-IS and ignores the rest, so sensor is a poor choice
// for a tracker despite sounding right.
#define DEFAULT_MESH_NODE_TYPE "chat"
// direct | flood. Direct still reaches APRS-IS - a repeater gates the
// position before it decides whether to forward - but does not make a
// moving station re-flood across the whole mesh.
#define DEFAULT_MESH_ROUTE     "direct"
#define DEFAULT_MESH_INTERVAL  900
// The RFC's 70 cm ham calling channel. Occupied ~434.859..434.921 MHz;
// nothing may be emitted above 435.000, which protects the
// amateur-satellite segment.
#define DEFAULT_MESH_FREQ      434.890f
#define DEFAULT_MESH_BW        62.5f
#define DEFAULT_MESH_SF        8
#define DEFAULT_MESH_CR        8
#define DEFAULT_MESH_PREAMBLE  16
// --- MeshCom (tx-only position beacon) ---
#define DEFAULT_MESHCOM_ENABLED    false
#define DEFAULT_MESHCOM_INTERVAL   900
#define DEFAULT_MESHCOM_SMART      false
#define DEFAULT_MESHCOM_MAX_HOP    2
// Upstream carries two hop limits, not one: initAPRS() gives ':' and '@'
// max_hop_text and everything else max_hop_pos. Every node on the air here
// shows the same split - positions at H02, messages and HEY at H04.
#define DEFAULT_MESHCOM_HOP_TEXT   4
// HEY, the "who hears me" probe, at the rate a normal node settles on.
// Upstream damps it with a trickle timer (docs/hey-supp.md: Imin 30 s,
// doubling to Imax 15 min, suppressed when two neighbours' HEYs were
// heard). A receiver is what makes that safe, and this board has none, so
// it can never suppress - running the fast end of the ramp would put it on
// the air more often than any real node. It sends at Imax only.
#define DEFAULT_MESHCOM_HEY        true
// The free-text segment of a position: comment, then the name behind a
// '#'. PositionToAPRS() builds it that way and a node with no comment
// still sends "#name", so the name is the part that carries.
#define DEFAULT_MESHCOM_NAME       ""
// Sent verbatim, 25 characters, and empty falls back to "Meshcom" - the
// position has to say which network it came out of, because both paths
// reach APRS-IS under the same callsign and nothing else there does. Not
// shared with the APRS comment: that one is written for a different map,
// is four times longer, and is usually a URL, whose ':' and '/' this
// field's filter removes.
#define DEFAULT_MESHCOM_COMMENT    ""
// Group subscriptions, sent as /R=9;262; at the very end of the position.
// Empty by default and legitimately so: PositionToAPRS() only emits the
// field when the node has at least one group, and this radio cannot
// receive, so declaring a group asks the server to route traffic at a
// station that will never hear it.
#define DEFAULT_MESHCOM_GROUPS     ""
#define MESHCOM_HEY_INTERVAL_MS    900000UL
#define DEFAULT_MESHCOM_FREQ       433.175f
#define DEFAULT_MESHCOM_BW         250.0f
#define DEFAULT_MESHCOM_SF         11
#define DEFAULT_MESHCOM_CR         6
// 8, not the 32 of MeshCom's DEFAULT_PREAMPLE_LENGTH. That constant is the
// "EU" country profile; the ham networks on this frequency run "EU8",
// whose entry in lora_setcountry() is labelled "EU Preabble 8" and sets
// node_preamplebits = 8. The two differ in nothing else. Sending 32 is
// received perfectly well - a longer preamble always is - but it makes the
// station the only one on the air with a different profile, and it says so
// out loud: the country nibble of the modulation byte reads 0 (EU) where
// every neighbour reads 8 (EU8). It also costs 24 symbols, 197 ms at
// SF11/250 kHz, on every single frame.
#define DEFAULT_MESHCOM_PREAMBLE   8
// The hardware id a receiving node looks up in its own table
// (mheard_functions.cpp getHardwareLong(): 1 TLORA_V2, 2 TLORA_V1,
// 3 TLORA_V2_1_1p6, 4 TBEAM, 7 T_ECHO, 8 TDECK, 9 RAK4631 ... and a remap
// for the ids above 38). This board is not in that table, so it borrows the
// closest plain SX127x node: id 0 is literally "no info", and a station that
// reports it is at best displayed as unknown hardware and at worst dropped
// by the gateways on the way to the map.
#define DEFAULT_MESHCOM_HWID       1
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

// fastRate is deliberately far slower than classic SmartBeaconing.
//
// Those constants come from 1200 baud AX.25, where a beacon is about half
// a second and a 15 s interval costs ~3% of the channel. This is SF12: an
// 82-byte frame is 3.45 s on air, so the same 15 s is **23%** of a shared
// channel from one tracker. Two of them on a motorway and half the band is
// gone.
//
// 60 s at 60 km/h is a position every kilometre, and corner pegging still
// fires on turns regardless - which is what it exists for - so very little
// tracking fidelity is lost for a fourfold cut in airtime.
static const SBPreset PRESET_CAR = {
    60, 180, 60, 10, 1.0f, 100, 3, 30, 5.0f, true
};
static const SBPreset PRESET_BIKE = {
    90, 240, 25, 5, 0.6f, 100, 3, 22, 4.0f, true
};
static const SBPreset PRESET_HIKER = {
    120, 360, 10, 1, 0.4f, 100, 3, 18, 3.0f, true
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
    int    metaGap;
    int    paWarmup;

    // MeshCore
    bool   aprsTimestamp;
    bool   aprsAltitude;
    int    commentInterval;

    bool   battProtect;
    int    battLowVoltage;
    int    battChargeVoltage;
    int    battPaDrive;
    int    battStartDelay;
    int    battStepVolts;
    int    battSettle;
    char   alertChannel[20];
    char   alertMention[16];
    char   alertAprsCall[16];
    char   alertMeshComCall[16];
    int    alertInterval;
    int    sbParkedAfter;
    int    sbParkedRate;

    bool   meshEnabled;
    char   meshName[24];
    char   meshNodeType[12];
    char   meshRoute[8];
    int    meshInterval;
    float  meshFrequency;
    float  meshBandwidth;
    int    meshSf;
    int    meshCr;
    int    meshPreamble;
    bool   meshComEnabled;
    int    meshComInterval;
    bool   meshComSmart;   // follow SmartBeacon instead of the interval
    int    meshComMaxHop;
    int    meshComHopText;
    bool   meshComHey;
    char   meshComName[16];
    char   meshComComment[32];
    char   meshComGroups[32];
    int    meshComHardwareId;
    float  meshComFrequency;
    float  meshComBandwidth;
    int    meshComSf;
    int    meshComCr;
    int    meshComPreamble;
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
    cfg.metaGap = DEFAULT_META_GAP;
    cfg.paWarmup = DEFAULT_PA_WARMUP;

    cfg.aprsTimestamp = DEFAULT_APRS_TIMESTAMP;
    cfg.aprsAltitude = DEFAULT_APRS_ALTITUDE;
    cfg.commentInterval = DEFAULT_COMMENT_INTERVAL;
    cfg.battProtect = DEFAULT_BATT_PROTECT;
    cfg.battLowVoltage = DEFAULT_BATT_LOW_V;
    cfg.battChargeVoltage = DEFAULT_BATT_CHARGE_V;
    cfg.battPaDrive = DEFAULT_BATT_PA_DRIVE;
    cfg.battStartDelay = DEFAULT_BATT_START_DELAY;
    cfg.battStepVolts = DEFAULT_BATT_STEP_V;
    cfg.battSettle = DEFAULT_BATT_SETTLE;
    strncpy(cfg.alertChannel, DEFAULT_ALERT_CHANNEL, sizeof(cfg.alertChannel));
    strncpy(cfg.alertMention, DEFAULT_ALERT_MENTION, sizeof(cfg.alertMention));
    strncpy(cfg.alertAprsCall, DEFAULT_ALERT_APRS, sizeof(cfg.alertAprsCall));
    strncpy(cfg.alertMeshComCall, DEFAULT_ALERT_MESHCOM, sizeof(cfg.alertMeshComCall));
    cfg.alertInterval = DEFAULT_ALERT_INTERVAL;
    cfg.sbParkedAfter = DEFAULT_SB_PARKED_AFTER;
    cfg.sbParkedRate = DEFAULT_SB_PARKED_RATE;

    cfg.meshEnabled = DEFAULT_MESH_ENABLED;
    strncpy(cfg.meshName, DEFAULT_MESH_NAME, sizeof(cfg.meshName));
    strncpy(cfg.meshNodeType, DEFAULT_MESH_NODE_TYPE, sizeof(cfg.meshNodeType));
    strncpy(cfg.meshRoute, DEFAULT_MESH_ROUTE, sizeof(cfg.meshRoute));
    cfg.meshInterval = DEFAULT_MESH_INTERVAL;
    cfg.meshFrequency = DEFAULT_MESH_FREQ;
    cfg.meshBandwidth = DEFAULT_MESH_BW;
    cfg.meshSf = DEFAULT_MESH_SF;
    cfg.meshCr = DEFAULT_MESH_CR;
    cfg.meshPreamble = DEFAULT_MESH_PREAMBLE;
    cfg.meshComEnabled = DEFAULT_MESHCOM_ENABLED;
    cfg.meshComInterval = DEFAULT_MESHCOM_INTERVAL;
    cfg.meshComSmart = DEFAULT_MESHCOM_SMART;
    cfg.meshComMaxHop = DEFAULT_MESHCOM_MAX_HOP;
    cfg.meshComHopText = DEFAULT_MESHCOM_HOP_TEXT;
    cfg.meshComHey = DEFAULT_MESHCOM_HEY;
    strncpy(cfg.meshComName, DEFAULT_MESHCOM_NAME, sizeof(cfg.meshComName) - 1);
    strncpy(cfg.meshComComment, DEFAULT_MESHCOM_COMMENT, sizeof(cfg.meshComComment) - 1);
    strncpy(cfg.meshComGroups, DEFAULT_MESHCOM_GROUPS, sizeof(cfg.meshComGroups) - 1);
    cfg.meshComHardwareId = DEFAULT_MESHCOM_HWID;
    cfg.meshComFrequency = DEFAULT_MESHCOM_FREQ;
    cfg.meshComBandwidth = DEFAULT_MESHCOM_BW;
    cfg.meshComSf = DEFAULT_MESHCOM_SF;
    cfg.meshComCr = DEFAULT_MESHCOM_CR;
    cfg.meshComPreamble = DEFAULT_MESHCOM_PREAMBLE;
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
    } else if (strcasecmp(key, "metaGap") == 0) {
        cfg.metaGap = constrain(atoi(val), 0, 10000);
    } else if (strcasecmp(key, "paWarmup") == 0) {
        cfg.paWarmup = constrain(atoi(val), 0, 5000);
    } else if (strcasecmp(key, "aprsTimestamp") == 0) {
        cfg.aprsTimestamp = (strcasecmp(val, "true") == 0 || strcmp(val, "1") == 0);
    } else if (strcasecmp(key, "aprsAltitude") == 0) {
        cfg.aprsAltitude = (strcasecmp(val, "true") == 0 || strcmp(val, "1") == 0);
    } else if (strcasecmp(key, "commentInterval") == 0) {
        // A word or a number of seconds, the same way loraTcxo takes
        // "auto" or a voltage. "always" is what an operator reaches for;
        // 0 is what it means. "off" is the other end and needs a word of
        // its own rather than a number, because 0 is already taken by its
        // opposite - "commentInterval=0" reading as "never" is exactly the
        // kind of near-miss that goes unnoticed in the field.
        if (strcasecmp(val, "always") == 0) {
            cfg.commentInterval = 0;
        } else if (strcasecmp(val, "off") == 0 || strcasecmp(val, "never") == 0 ||
                   strcasecmp(val, "none") == 0) {
            cfg.commentInterval = COMMENT_INTERVAL_OFF;
        } else {
            cfg.commentInterval = constrain(atoi(val), 0, 86400);
        }
    } else if (strcasecmp(key, "battProtect") == 0) {
        cfg.battProtect = (strcasecmp(val, "true") == 0 || strcmp(val, "1") == 0);
    } else if (strcasecmp(key, "battLowVoltage") == 0) {
        // Nothing below 1200 is measurable - the ADC scale bottoms out
        // there - so a lower setting would simply never fire.
        cfg.battLowVoltage = constrain(atoi(val), 1200, 1350);
    } else if (strcasecmp(key, "battChargeVoltage") == 0) {
        cfg.battChargeVoltage = constrain(atoi(val), 1250, 1500);
    } else if (strcasecmp(key, "battPaDrive") == 0) {
        cfg.battPaDrive = constrain(atoi(val), -9, 22);
    } else if (strcasecmp(key, "battStartDelay") == 0) {
        cfg.battStartDelay = constrain(atoi(val), 0, 600);
    } else if (strcasecmp(key, "battStepVolts") == 0) {
        cfg.battStepVolts = constrain(atoi(val), 0, 300);
    } else if (strcasecmp(key, "battSettle") == 0) {
        cfg.battSettle = constrain(atoi(val), 0, 60);
    } else if (strcasecmp(key, "alertChannel") == 0) {
        strncpy(cfg.alertChannel, val, sizeof(cfg.alertChannel) - 1);
    } else if (strcasecmp(key, "alertMention") == 0) {
        strncpy(cfg.alertMention, val, sizeof(cfg.alertMention) - 1);
    } else if (strcasecmp(key, "alertAprsCall") == 0) {
        strncpy(cfg.alertAprsCall, val, sizeof(cfg.alertAprsCall) - 1);
        for (char *q = cfg.alertAprsCall; *q; q++) *q = toupper(*q);
    } else if (strcasecmp(key, "alertMeshComCall") == 0) {
        strncpy(cfg.alertMeshComCall, val, sizeof(cfg.alertMeshComCall) - 1);
        for (char *q = cfg.alertMeshComCall; *q; q++) *q = toupper(*q);
    } else if (strcasecmp(key, "alertInterval") == 0) {
        cfg.alertInterval = constrain(atoi(val), 10, 3600);
    } else if (strcasecmp(key, "sbParkedAfter") == 0) {
        cfg.sbParkedAfter = constrain(atoi(val), 0, 86400);
    } else if (strcasecmp(key, "sbParkedRate") == 0) {
        cfg.sbParkedRate = constrain(atoi(val), 60, 3600);
    } else if (strcasecmp(key, "meshEnabled") == 0) {
        cfg.meshEnabled = (strcasecmp(val, "true") == 0 || strcmp(val, "1") == 0);
    } else if (strcasecmp(key, "meshName") == 0) {
        strncpy(cfg.meshName, val, sizeof(cfg.meshName) - 1);
    } else if (strcasecmp(key, "meshNodeType") == 0) {
        strncpy(cfg.meshNodeType, val, sizeof(cfg.meshNodeType) - 1);
    } else if (strcasecmp(key, "meshRoute") == 0) {
        strncpy(cfg.meshRoute, val, sizeof(cfg.meshRoute) - 1);
    } else if (strcasecmp(key, "meshInterval") == 0) {
        cfg.meshInterval = constrain(atoi(val), 60, 86400);
    } else if (strcasecmp(key, "meshFrequency") == 0) {
        // Never above 435.000 - the amateur-satellite segment starts there
        // and the occupied bandwidth reaches ~31 kHz above the carrier.
        cfg.meshFrequency = constrain((float)atof(val), 430.0f, 434.960f);
    } else if (strcasecmp(key, "meshBandwidth") == 0) {
        // The one radio key without validation was the one reachable way to
        // fail a retune partway and strand the radio off-channel. Only the
        // bandwidths RadioLib accepts are allowed through.
        float bw = (float)atof(val);
        const float ok[] = { 7.8f, 10.4f, 15.6f, 20.8f, 31.25f,
                             41.7f, 62.5f, 125.0f, 250.0f, 500.0f };
        cfg.meshBandwidth = DEFAULT_MESH_BW;
        for (unsigned i = 0; i < sizeof(ok) / sizeof(ok[0]); i++) {
            if (fabsf(bw - ok[i]) < 0.01f) { cfg.meshBandwidth = bw; break; }
        }
    } else if (strcasecmp(key, "meshSf") == 0) {
        // 6, not 5: RadioLib's SX1278 switch starts at 6, so SF5 would fail
        // the retune on a V1 board even though an SX126x would take it.
        cfg.meshSf = constrain(atoi(val), 6, 12);
    } else if (strcasecmp(key, "meshCr") == 0) {
        cfg.meshCr = constrain(atoi(val), 5, 8);
    } else if (strcasecmp(key, "meshPreamble") == 0) {
        cfg.meshPreamble = constrain(atoi(val), 6, 64);
    } else if (strcasecmp(key, "meshComEnabled") == 0) {
        cfg.meshComEnabled = (strcasecmp(val, "true") == 0 || strcmp(val, "1") == 0);
    } else if (strcasecmp(key, "meshComInterval") == 0) {
        // A number of seconds, or "smart" to hang the mesh position off the
        // same SmartBeacon decision that drives APRS - the same way
        // commentInterval takes "always". "smart" ignores this number and
        // uses MESHCOM_SMART_FLOOR_S as its floor instead, so a vehicle
        // beaconing every 60 s in traffic cannot flood the mesh.
        if (strcasecmp(val, "smart") == 0) {
            cfg.meshComSmart = true;
        } else {
            cfg.meshComSmart = false;
            cfg.meshComInterval = constrain(atoi(val), 60, 86400);
        }
    } else if (strcasecmp(key, "meshComMaxHop") == 0) {
        cfg.meshComMaxHop = constrain(atoi(val), 0, 7);
    } else if (strcasecmp(key, "meshComHopText") == 0) {
        cfg.meshComHopText = constrain(atoi(val), 0, 7);
    } else if (strcasecmp(key, "meshComName") == 0) {
        strncpy(cfg.meshComName, val, sizeof(cfg.meshComName) - 1);
    } else if (strcasecmp(key, "meshComComment") == 0) {
        strncpy(cfg.meshComComment, val, sizeof(cfg.meshComComment) - 1);
    } else if (strcasecmp(key, "meshComGroups") == 0) {
        strncpy(cfg.meshComGroups, val, sizeof(cfg.meshComGroups) - 1);
    } else if (strcasecmp(key, "meshComHey") == 0) {
        cfg.meshComHey = (strcasecmp(val, "true") == 0 || strcmp(val, "1") == 0);
    } else if (strcasecmp(key, "meshComHardwareId") == 0) {
        cfg.meshComHardwareId = constrain(atoi(val), 0, 127);
    } else if (strcasecmp(key, "meshComFrequency") == 0) {
        cfg.meshComFrequency = constrain((float)atof(val), 430.0f, 434.960f);
    } else if (strcasecmp(key, "meshComBandwidth") == 0) {
        float bw = (float)atof(val);
        const float ok[] = { 7.8f, 10.4f, 15.6f, 20.8f, 31.25f,
                             41.7f, 62.5f, 125.0f, 250.0f, 500.0f };
        cfg.meshComBandwidth = DEFAULT_MESHCOM_BW;
        for (unsigned i = 0; i < sizeof(ok) / sizeof(ok[0]); i++) {
            if (fabsf(bw - ok[i]) < 0.01f) { cfg.meshComBandwidth = bw; break; }
        }
    } else if (strcasecmp(key, "meshComSf") == 0) {
        cfg.meshComSf = constrain(atoi(val), 6, 12);
    } else if (strcasecmp(key, "meshComCr") == 0) {
        cfg.meshComCr = constrain(atoi(val), 5, 8);
    } else if (strcasecmp(key, "meshComPreamble") == 0) {
        cfg.meshComPreamble = constrain(atoi(val), 6, 64);
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
    "# Boot-burst diagnostics: ms between the three metadata frames, and ms\n"
    "# of unmodulated carrier before the first one. Leave at 150 and 0.\n"
    "metaGap=150\n"
    "paWarmup=0\n"
    "\n"
    "# --- Battery (thresholds measured on a vehicle) ---\n"
    "# Arms itself only after it has seen the alternator, so a tracker on\n"
    "# USB or a bench supply never throttles itself.\n"
    "battProtect=true\n"
    "battLowVoltage=1200\n"
    "battChargeVoltage=1300\n"
    "battPaDrive=10\n"
    "# 0 for an EV - no starter motor, so nothing to ride out\n"
    "battStartDelay=10\n"
    "# hold off transmitting while the rail is moving (crank dip and rise)\n"
    "battStepVolts=30\n"
    "battSettle=5\n"
    "\n"
    "# --- Alerts (no console in a car - put the diagnostics on the air) ---\n"
    "# Both empty = off. alertChannel is a MeshCore public channel.\n"
    "alertChannel=\n"
    "alertMention=\n"
    "alertAprsCall=\n"
    "alertMeshComCall=\n"
    "alertInterval=60\n"
    "\n"
    "# --- Parked ---\n"
    "# Stationary this long (s) and the beacon rate drops. 0 = off.\n"
    "sbParkedAfter=14400\n"
    "sbParkedRate=1800\n"
    "\n"
    "# --- Airtime ---\n"
    "# At SF12 a frame is ~3.5s on air. Each of these saves ~10%.\n"
    "# timestamp off sends '!' instead of '@ddhhmmz'; receivers stamp\n"
    "# on arrival anyway, and this tracker cannot receive messages.\n"
    "aprsTimestamp=true\n"
    "aprsAltitude=true\n"
    "# Seconds between comments, \"always\", or \"off\" for none at all.\n"
    "commentInterval=1800\n"
    "\n"
    "# --- MeshCore (IARU R1 ham profile, 70cm) ---\n"
    "# Mints an Ed25519 identity in /meshid.bin on first enable.\n"
    "meshEnabled=false\n"
    "meshName=NOCALL\n"
    "# chat or repeater are gated to APRS-IS; sensor and room are not\n"
    "meshNodeType=chat\n"
    "# direct still reaches APRS-IS without re-flooding the mesh\n"
    "meshRoute=direct\n"
    "meshInterval=900\n"
    "\n"
    "# --- MeshCom (tx-only position beacon) ---\n"
    "meshComEnabled=false\n"
    "meshComInterval=900\n"
    "meshComMaxHop=2\n"
    "meshComHopText=4\n"
    "meshComHey=true\n"
    "meshComName=\n"
    "meshComComment=\n"
    "meshComGroups=\n"
    "meshComHardwareId=1\n"
    "meshComFrequency=433.175\n"
    "meshComBandwidth=250.0\n"
    "meshComSf=11\n"
    "meshComCr=6\n"
    "meshComPreamble=8\n"
    "\n"
    "# --- Debug ---\n"
    "fullDebug=false\n";

static bool configCreateDefault() {
    File f = FatFS.open("/config.txt", "w");
    if (!f) {
        Serial.println("[CONFIG] ERROR: Could not create file");
        return false;
    }
    // Write line by line to avoid large buffer issues. The watchdog is fed
    // along the way: ninety lines is several sector programs, and on a
    // fresh board this ran past its 8 s and left a file with a size of 0
    // over clusters full of text - which then hung every boot after.
    watchdog_update();
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
    watchdog_update();
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
    f.println("# Boot-burst diagnostics: ms between the three metadata frames, and ms");
    f.println("# of unmodulated carrier before the first one. Leave at 150 and 0.");
    f.println("metaGap=150");
    f.println("paWarmup=0");
    f.println("");
    watchdog_update();
    f.println("# --- Battery (thresholds measured on a vehicle) ---");
    f.println("# Arms only after it has seen the alternator, so a tracker on");
    f.println("# USB or a bench supply never throttles itself.");
    f.println("battProtect=true");
    f.println("battLowVoltage=1200");
    f.println("battChargeVoltage=1300");
    f.println("battPaDrive=10");
    f.println("# 0 for an EV - no starter motor, so nothing to ride out");
    f.println("battStartDelay=10");
    f.println("# hold off transmitting while the rail is moving (crank dip and rise)");
    f.println("battStepVolts=30");
    f.println("battSettle=5");
    f.println("");
    f.println("# --- Alerts (no console in a car - put the diagnostics on the air) ---");
    f.println("# Both empty = off. alertChannel is a MeshCore public channel.");
    f.println("alertChannel=");
    f.println("alertMention=");
    f.println("alertAprsCall=");
    f.println("# MeshCom destination for alerts: a callsign, a group number, or *");
    f.println("alertMeshComCall=");
    f.println("alertInterval=60");
    f.println("");
    f.println("# --- Parked ---");
    f.println("# Stationary this long (s) and the beacon rate drops. 0 = off.");
    f.println("sbParkedAfter=14400");
    f.println("sbParkedRate=1800");
    f.println("");
    watchdog_update();
    f.println("# --- Airtime ---");
    f.println("# At SF12 a frame is ~3.5s on air. Each of these saves ~10%.");
    f.println("# timestamp off sends '!' instead of '@ddhhmmz'.");
    f.println("aprsTimestamp=true");
    f.println("aprsAltitude=true");
    f.println("# Seconds between comments, \"always\", or \"off\" for none.");
    f.println("commentInterval=1800");
    f.println("");
    watchdog_update();
    f.println("# --- MeshCore (IARU R1 ham profile, 70cm) ---");
    f.println("# Mints an Ed25519 identity in /meshid.bin on first enable.");
    f.println("meshEnabled=false");
    f.println("meshName=NOCALL");
    f.println("# chat or repeater are gated to APRS-IS; sensor and room are not");
    f.println("meshNodeType=chat");
    f.println("# direct still reaches APRS-IS without re-flooding the mesh");
    f.println("meshRoute=direct");
    f.println("meshInterval=900");
    f.println("");
    f.println("# --- MeshCom (tx-only position beacon) ---");
    f.println("meshComEnabled=false");
    f.println("# seconds, or \"smart\" to follow the APRS SmartBeacon");
    f.println("meshComInterval=900");
    f.println("meshComMaxHop=2");
    f.println("# hop count for messages and HEY, as upstream splits them");
    f.println("meshComHopText=4");
    f.println("# the \"who hears me\" probe every 15 min, like a normal node");
    f.println("meshComHey=true");
    f.println("# station name, sent as #name - every real node has one");
    f.println("meshComName=");
    f.println("# free text, max 25 chars; avoid spaces, receivers cut there");
    f.println("# sent verbatim, e.g. vMesh; empty falls back to \"Meshcom\"");
    f.println("meshComComment=");
    f.println("# group subscriptions, e.g. 9;262; - see the README before setting");
    f.println("meshComGroups=");
    f.println("# 1=TLORA_V2 - this board is not in MeshCom's own table");
    f.println("meshComHardwareId=1");
    f.println("meshComFrequency=433.175");
    f.println("meshComBandwidth=250.0");
    f.println("meshComSf=11");
    f.println("meshComCr=6");
    f.println("meshComPreamble=8");
    f.println("");
    f.println("fullDebug=false");
    f.close();
    watchdog_update();
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

    // A config.txt whose first-boot write was cut short - a directory
    // entry of size 0 over clusters that still hold the text - reads at
    // milliseconds per byte and never reaches end of file, and a board
    // that hangs here reboots on the watchdog forever. So the read is
    // bounded three ways: read() < 0 is end of file (the old `char c`
    // turned -1 into 0xFF and looped), the byte count is capped, and the
    // whole load has a time budget a healthy file clears in milliseconds.
    // Blowing it means the filesystem is what is broken.
    const unsigned long budgetMs = 3000;
    const size_t        maxBytes = 16384;
    unsigned long       t0       = millis();
    size_t              total    = 0;
    bool                corrupt  = false;

    char line[128];
    while (f.available()) {
        int len = 0;
        memset(line, 0, sizeof(line));
        while (f.available() && len < (int)sizeof(line) - 1) {
            int c = f.read();
            if (c < 0) break;
            total++;
            if (c == '\n') break;
            line[len++] = (char)c;
        }
        watchdog_update();
        if (total > maxBytes || (millis() - t0) > budgetMs) {
            corrupt = true;
            break;
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

    if (corrupt) {
        Serial.printf("[CONFIG] config.txt unreadable (%u bytes in %lu ms) - "
                      "reformatting the filesystem and starting over\n",
                      (unsigned)total, millis() - t0);
        // This loses /meshid.bin too. A board stuck in a boot loop has no
        // identity worth keeping, so that is the lesser loss.
        FatFS.end();
        watchdog_update();
        FatFS.format();
        watchdog_update();
        FatFS.begin();
        configCreateDefault();
        Serial.println("[CONFIG] Rebooting on the fresh filesystem");
        delay(200);
        rp2040.reboot();
        while (true) { }
    }

    configApplyProfile(cfg);

    Serial.printf("[CONFIG] Loaded: callsign=%s profile=%s power=%d freq=%.3f\n",
                  cfg.callsign, cfg.profile, cfg.power, cfg.loraFrequency);
    return true;
}
