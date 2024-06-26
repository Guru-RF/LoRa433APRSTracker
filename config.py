fullDebug = False
power = 23  # min 5dBm max 23dBm
hasPa = True  # has a power amplifier (adds 6dBm)
callsign = "--CALL--"
# symbol = '/b' # bike
# symbol = '/>' # car
# symbol = '\>' # car
symbol = "L>"  # lora car
# symbol = 'Uk' # truck
# symbol = '/j' # jeep
# symbol = '/v' # truck
# symbol = '/u' # 18 wheeler
# symbol = '/U' # bus
comment = "https://RF.Guru"
rate = 15  # seconds (packet frequency when moving)
keepalive = 300  # in seconds (When no movement report every ...) when on solar/battary set this 900 (15 mins) else 300 (5 mins)
distance = 100  # minimum distance change to report
voltage = True  # monitor voltage
triggerVoltage = True
triggerVoltageLevel = (
    1200  # bellow will trigger an alert (APRS message to triggerVoltageCall)
)
triggerVoltageCall = "--MSGS--"
triggerVoltageKeepalive = (
    3600  # an hour, this will send warning messages until voltage levels stabalize!
)
i2cEnabled = True
i2cDevices = ["BME680"]
bme680_tempOffset = 0
