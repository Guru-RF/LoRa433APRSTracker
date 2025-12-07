# -------------------------------
# RF.Guru LoRa APRS Tracker Config
# -------------------------------

fullDebug = False

# LoRa RF power
power = 23  # min 5dBm max 23dBm

# External power amplifier?
hasPa = True  # adds ~6dBm

# APRS callsign & symbol
callsign = "--CALL--"
symbol = "L>"            # car (LoRa)
# symbol examples:
# '/b' bike, '/>' car, '/<' motorbike, 'Uk' truck, '/U' bus, etc.

# APRS comment
comment = "https://RF.Guru"

# Basic beaconing configuration (legacy mode)
rate = 15            # seconds (movement interval, if SmartBeaconing disabled)
keepalive = 300      # report interval when not moving
distance = 100       # meters threshold for movement beacon

# Voltage monitoring
voltage = True
triggerVoltage = True
triggerVoltageLevel = 1200       # mV × 100 (12.00 V)
triggerVoltageCall = "--MSGS--"
triggerVoltageKeepalive = 3600   # seconds

# I2C sensor support
i2cEnabled = True
i2cDevices = ["BME680"]
bme680_tempOffset = 0


# ------------------------------------------------
# SmartBeaconing (Advanced APRS Path Optimization)
# ------------------------------------------------
# Set smartBeaconing = False to disable completely
smartBeaconing = True


# Fast beacon rate (when moving quickly)
sb_fastRate = 15          # seconds (typical 10–20)

# Slow beacon rate (when moving slowly)
sb_slowRate = 180         # seconds (typical 120–300)

# Speed thresholds (km/h)
sb_fastSpeed = 60         # above → use fastRate
sb_slowSpeed = 10         # below → use slowRate

# Minimum speed to be considered "moving"
sb_stationarySpeed = 1.0  # km/h → below this freezes GPS drift

# Heading change needed to trigger corner-peg beacon
sb_turnThreshold = 30     # degrees (28–35 common)

# Additional heading change per second for dynamic pegging
sb_turnSlope = 5          # deg/sec, optional

# Smooth heading filter (prevents jitter)
sb_headingFilter = True   # True / False
