# ------------------------------------------------------------
# RF.Guru LoRa APRS Tracker Configuration (SmartBeaconing Only)
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

# APRS identification
callsign = "--CALL--"
symbol   = "L>"             # LoRa car symbol (change per profile if desired)
comment  = "https://RF.Guru"

# Voltage monitoring
voltage = True
triggerVoltage = True
triggerVoltageLevel = 1200         # mV x100 → 12.00V threshold
triggerVoltageCall = "--MSGS--"
triggerVoltageKeepalive = 3600     # seconds


# I2C sensors
i2cEnabled = True
i2cDevices = ["BME680"]
bme680_tempOffset = 0



# ------------------------------------------------------------
# SMARTBEACONING PRESETS
# ------------------------------------------------------------
# Each preset contains settings tuned for movement pattern.


PRESET_CAR = {
    "fastRate": 15,          # Fast road speed beaconing
    "slowRate": 180,         # Low speed / city driving
    "fastSpeed": 60,         # Over 60 km/h → fast beaconing
    "slowSpeed": 10,         # Under 10 km/h → slow beaconing
    "stationarySpeed": 1.0,  # Below → freeze GPS
    "turnThreshold": 30,     # Corner peg at 30°
    "turnSlope": 5,          # Dynamic turn trigger
    "headingFilter": True
}

PRESET_BIKE = {
    "fastRate": 25,          # Bikes move slower
    "slowRate": 240,         # Allow longer intervals
    "fastSpeed": 25,         # Above 25 km/h considered fast
    "slowSpeed": 5,          # Under 5 km/h → slow
    "stationarySpeed": 0.6,  # Slight GPS drift at low speed
    "turnThreshold": 22,     # Bikes make sharper turns
    "turnSlope": 4,          # More sensitive to rotation
    "headingFilter": True
}

PRESET_HIKER = {
    "fastRate": 40,          # Very slow movement
    "slowRate": 360,         # Up to 6 minutes when slow
    "fastSpeed": 10,         # Over 10 km/h for hikers = fast
    "slowSpeed": 1.5,        # Below this considered slow
    "stationarySpeed": 0.4,  # Freeze at very low movement
    "turnThreshold": 18,     # Sensitive to direction changes
    "turnSlope": 3,          # Hikers turn often
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
