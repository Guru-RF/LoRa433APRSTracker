
# ------------------------------------------------------------
# RF.Guru LoRa APRS Tracker Configuration
# (SmartBeaconing + Telemetry + Voltage + optional I2C)
# ------------------------------------------------------------

# Select profile: "car", "bike", "hiker"
profile = "car"     # <-- DEFAULT

# ------------------------------------------------------------
# Common Settings
# ------------------------------------------------------------
fullDebug = False

# Radio settings
power = 23          # 5–23 dBm
hasPa = True        # PA adds ~6 dBm

# Optional override
# loraFrequency = 433.775

# APRS identification
callsign = "N0CALL-7"     # <-- CHANGE THIS
symbol   = "L>"           # 2 chars: symbol table + symbol
comment  = "https://RF.Guru"

# ------------------------------------------------------------
# Legacy compatibility (only used when smartBeaconing=False)
# ------------------------------------------------------------
rate = 30          # seconds
keepalive = 900    # seconds
distance = 50      # meters

# ------------------------------------------------------------
# Voltage monitoring
# ------------------------------------------------------------
voltage = True
triggerVoltage = True
triggerVoltageLevel = 1200         # V*100 → 12.00V threshold
triggerVoltageCall = "N0CALL"      # <-- destination addressee for alert (max 9 chars)
triggerVoltageKeepalive = 3600     # seconds (min time between voltage alert messages)

# ------------------------------------------------------------
# I2C sensors
# ------------------------------------------------------------
i2cEnabled = True
i2cDevices = ["BME680"]            # supported: "BME680" or "SHTC3"
bme680_tempOffset = 0              # add offset in °C if needed

# ------------------------------------------------------------
# SMARTBEACONING PRESETS
# ------------------------------------------------------------

PRESET_CAR = {
    "fastRate": 15,          # seconds at/above fastSpeed
    "slowRate": 180,         # seconds at/below slowSpeed or stationary
    "fastSpeed": 60,         # km/h
    "slowSpeed": 10,         # km/h
    "stationarySpeed": 1.0,  # km/h (below -> position freeze)
    "turnThreshold": 30,     # degrees
    "turnSlope": 5,          # deg/sec (turn rate trigger)
    "headingFilter": True
}

PRESET_BIKE = {
    "fastRate": 25,
    "slowRate": 240,
    "fastSpeed": 25,
    "slowSpeed": 5,
    "stationarySpeed": 0.6,
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
    "turnThreshold": 18,
    "turnSlope": 3,
    "headingFilter": True
}

# ------------------------------------------------------------
# APPLY PROFILE
# ------------------------------------------------------------

if profile.lower() == "car":
    p = PRESET_CAR
elif profile.lower() == "bike":
    p = PRESET_BIKE
elif profile.lower() == "hiker":
    p = PRESET_HIKER
else:
    raise ValueError("Unknown profile in config.py → choose car / bike / hiker")

# Export applied SmartBeaconing settings
smartBeaconing = True
sb_fastRate = p["fastRate"]
sb_slowRate = p["slowRate"]
sb_fastSpeed = p["fastSpeed"]
sb_slowSpeed = p["slowSpeed"]
sb_stationarySpeed = p["stationarySpeed"]
sb_turnThreshold = p["turnThreshold"]
sb_turnSlope = p["turnSlope"]
sb_headingFilter = p["headingFilter"]
