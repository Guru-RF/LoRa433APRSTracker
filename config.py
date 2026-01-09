# ------------------------------------------------------------
# RF.Guru LoRa APRS Tracker Configuration
# ------------------------------------------------------------

profile = "car"

fullDebug = False

power = 23
hasPa = True

callsign = "--CALL--"
symbol = "L>"
comment = "https://RF.Guru"

voltage = True
triggerVoltage = True
triggerVoltageLevel = 1200
triggerVoltageCall = "--MSGS--"
triggerVoltageKeepalive = 3600

i2cEnabled = True
i2cDevices = ["BME680"]
bme680_tempOffset = 0


# ------------------------------------------------------------
# GPS LED / lock behavior (v1 style + logging)
# ------------------------------------------------------------
gpsBlinkInterval = 2.0        # seconds between flashes while no fix
gpsBlinkPulse = 0.10          # seconds LED stays on per flash
gpsNoFixLogInterval = 5.0     # print "acquiring lock" every N seconds

gpsLockHold = 0.0             # seconds continuous fix required before "GPS FIX acquired"
gpsUnlockHold = 2.0           # seconds continuous no-fix before "GPS FIX lost"


# ------------------------------------------------------------
# SMARTBEACONING PRESETS
# ------------------------------------------------------------

PRESET_CAR = {
    "fastRate": 15,
    "slowRate": 180,
    "fastSpeed": 60,
    "slowSpeed": 10,
    "stationarySpeed": 1.0,

    # Stationary jitter lock (meters + consecutive fixes)
    "stationaryDistance": 100,
    "stationaryExitCount": 3,

    "turnThreshold": 30,
    "turnSlope": 5,
    "headingFilter": True
}

PRESET_BIKE = {
    "fastRate": 25,
    "slowRate": 240,
    "fastSpeed": 25,
    "slowSpeed": 5,
    "stationarySpeed": 0.6,

    "stationaryDistance": 100,
    "stationaryExitCount": 3,

    "turnThreshold": 22,
    "turnSlope": 4,
    "headingFilter": True
}

PRESET_HIKER = {
    "fastRate": 40,
    "slowRate": 360,
    "fastSpeed": 10,
    "slowSpeed": 1.5,
    "stationarySpeed": 0.4,

    "stationaryDistance": 100,
    "stationaryExitCount": 3,

    "turnThreshold": 18,
    "turnSlope": 3,
    "headingFilter": True
}

if profile.lower() == "car":
    p = PRESET_CAR
elif profile.lower() == "bike":
    p = PRESET_BIKE
elif profile.lower() == "hiker":
    p = PRESET_HIKER
else:
    raise ValueError("Unknown profile (car/bike/hiker)")

smartBeaconing = True
sb_fastRate = p["fastRate"]
sb_slowRate = p["slowRate"]
sb_fastSpeed = p["fastSpeed"]
sb_slowSpeed = p["slowSpeed"]
sb_stationarySpeed = p["stationarySpeed"]
sb_stationaryDistance = p["stationaryDistance"]
sb_stationaryExitCount = p["stationaryExitCount"]
sb_turnThreshold = p["turnThreshold"]
sb_turnSlope = p["turnSlope"]
sb_headingFilter = p["headingFilter"]
