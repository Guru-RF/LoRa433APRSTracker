import binascii
import os
import time
from math import atan2, ceil, cos, log, radians, sin, sqrt

import adafruit_gps
import adafruit_rfm9x
import board
import busio
import digitalio
import microcontroller
import rtc
import supervisor
import usb_cdc
from analogio import AnalogIn
from APRS import APRS
from microcontroller import watchdog as w
from rfguru_nvm import NonVolatileMemory
from watchdog import WatchDogMode

import config

# ============================================================
#       SYSTEM STARTUP
# ============================================================

supervisor.runtime.autoreload = False

# Watchdog
w.timeout = 5
w.mode = WatchDogMode.RESET
w.feed()

# Serial interface
serial = usb_cdc.data

# Small delay for console
time.sleep(3)
w.feed()

VERSION = "RF.Guru_LoRa433APRSTracker 1.0-SB"


# ============================================================
#       HELPERS
# ============================================================

def file_or_dir_exists(filename):
    try:
        return os.stat(filename)
    except OSError:
        return False


def year():
    return int(time.localtime().tm_year)


def purple(data):
    stamp = "{}".format(_format_datetime(time.localtime()))
    return "\x1b[38;5;104m[" + str(stamp) + "] " + data + "\x1b[0m"


def yellow(data):
    return "\x1b[38;5;220m" + data + "\x1b[0m"


def red(data):
    return "\x1b[1;5;31m" + data + "\x1b[0m"


def green(data):
    return "\x1b[1;5;32m" + data + "\x1b[0m"


# Geometry — distance in meters
def distance(lat1, lon1, lat2, lon2):
    if lat1 is None or lon1 is None:
        return 999999
    radius = 6371  # km
    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = sin(dlat/2)**2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon/2)**2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return int(radius * c * 1000)


# Voltage reader
def get_voltage(pin):
    if config.hasPa:
        return ((pin.value * 3.3) / 65536) + 10.6 + 1.4
    else:
        return ((pin.value * 3.3) / 65536) * 2


# Telemetry encoder
def base91_encode(number):
    if number < 0:
        raise ValueError("Only positive integers")
    if number == 0:
        return "!!"

    text = []
    max_n = ceil(log(number) / log(91))

    for n in range(int(max_n), -1, -1):
        quotient, number = divmod(number, 91**n)
        text.append(chr(33 + quotient))

    text = "".join(text).lstrip("!")
    if len(text) == 1:
        text = "!" + text

    return text


def _format_datetime(datetime):
    return "{:02}/{:02}/{} {:02}:{:02}:{:02}".format(
        datetime.tm_mon,
        datetime.tm_mday,
        datetime.tm_year,
        datetime.tm_hour,
        datetime.tm_min,
        datetime.tm_sec,
    )


# ============================================================
#        CONFIG VALIDATION
# ============================================================

try:
    if not isinstance(config.fullDebug, bool):
        raise AttributeError("fullDebug must be True/False")

    if not (5 <= config.power <= 23):
        raise AttributeError("power must be between 5 and 23 dBm")

    if not isinstance(config.hasPa, bool):
        raise AttributeError("hasPa must be True/False")

    if isinstance(config.callsign, bytes):
        raise AttributeError("callsign must be a string")

    if isinstance(config.symbol, bytes):
        raise AttributeError("symbol must be a string")

    if isinstance(config.comment, bytes):
        raise AttributeError("comment must be a string")

    if not (10 <= config.rate <= 60):
        raise AttributeError("rate must be 10–60 sec")

    if not (300 <= config.keepalive <= 3600):
        raise AttributeError("keepalive must be 300–3600 sec")

    if not (25 <= config.distance <= 1000):
        raise AttributeError("distance must be 25–1000 m")

    if not isinstance(config.voltage, bool):
        raise AttributeError("voltage must be True/False")

    if not isinstance(config.triggerVoltage, bool):
        raise AttributeError("triggerVoltage must be True/False")

    if not (250 <= config.triggerVoltageLevel <= 1280):
        raise AttributeError("triggerVoltageLevel must be 250–1280")

    if isinstance(config.triggerVoltageCall, bytes):
        raise AttributeError("triggerVoltageCall must be a string")

    if not (3600 <= config.triggerVoltageKeepalive <= 14400):
        raise AttributeError("triggerVoltageKeepalive must be 3600–14400")

    if not isinstance(config.i2cEnabled, bool):
        raise AttributeError("i2cEnabled must be True/False")

    if type(config.i2cDevices) not in (list, tuple):
        raise AttributeError("i2cDevices must be list or tuple")

    if not (0 <= config.bme680_tempOffset <= 99):
        raise AttributeError("bme680_tempOffset must be 0–99")

    # --- SmartBeaconing validation ---
    if not isinstance(config.smartBeaconing, bool):
        raise AttributeError("smartBeaconing must be True/False")

    if not (5 <= config.sb_fastRate <= 120):
        raise AttributeError("sb_fastRate must be 5–120 sec")

    if not (30 <= config.sb_slowRate <= 900):
        raise AttributeError("sb_slowRate must be 30–900 sec")

    if not (1 <= config.sb_stationarySpeed <= 10):
        raise AttributeError("sb_stationarySpeed must be 1–10 km/h")

    if not (5 <= config.sb_turnThreshold <= 90):
        raise AttributeError("sb_turnThreshold must be 5–90 degrees")

    if not (0 <= config.sb_turnSlope <= 20):
        raise AttributeError("sb_turnSlope must be 0–20 deg/sec")

    if not isinstance(config.sb_headingFilter, bool):
        raise AttributeError("sb_headingFilter must be True/False")

except AttributeError as error:
    print(red("CONFIG ERROR: " + str(error)))
    while True:
        w.feed()
        time.sleep(1)


# Final normalization
config.callsign = config.callsign.upper()


# ============================================================
#        SMARTBEACONING ENGINE
# ============================================================

class SmartBeacon:
    """
    Implements:
      - Dynamic fast/slow beaconing
      - Corner pegging
      - Stationary freeze
      - Heading smoothing
    """

    def __init__(self):
        self.last_beacon_time = time.monotonic()
        self.last_heading = None
        self.filtered_heading = None
        self.last_lat = None
        self.last_lon = None

    def heading_change(self, new_heading, dt):
        if self.last_heading is None:
            self.last_heading = new_heading
            return 0

        diff = abs(new_heading - self.last_heading)
        if diff > 180:
            diff = 360 - diff

        # Heading slope — degrees per second
        slope = diff / max(dt, 0.1)

        # Save for next cycle
        self.last_heading = new_heading

        return diff, slope

    def filter_heading(self, heading):
        if not config.sb_headingFilter:
            return heading

        if self.filtered_heading is None:
            self.filtered_heading = heading
            return heading

        # Low-pass filter
        alpha = 0.3
        self.filtered_heading = (alpha * heading +
                                 (1 - alpha) * self.filtered_heading)
        return self.filtered_heading

    def should_beacon(self, lat, lon, speed, heading):
        """
        Returns True if a beacon should be sent now.
        """

        now = time.monotonic()
        dt = now - self.last_beacon_time

        # Freeze jitter when nearly stationary
        if speed < config.sb_stationarySpeed:
            if dt >= config.sb_slowRate:
                return True
            return False

        # Fast/slow intervals
        interval = config.sb_fastRate if speed >= config.sb_fastSpeed else config.sb_slowRate

        # Basic timed beacon
        if dt >= interval:
            return True

        # Corner pegging
        if heading is not None:
            filtered = self.filter_heading(heading)
            diff, slope = self.heading_change(filtered, dt)

            if diff >= config.sb_turnThreshold:
                return True

            if slope >= config.sb_turnSlope:
                return True

        return False

    def update_after_beacon(self, lat, lon):
        self.last_beacon_time = time.monotonic()
        self.last_lat = lat
        self.last_lon = lon


# Create SmartBeacon engine instance
SB = SmartBeacon()

print(red(" -- Tracker Booted: " + config.callsign + " -=- " + VERSION))
print(yellow("SmartBeaconing: " + str(config.smartBeaconing)))

# ============================================================
#       INITIALIZE HARDWARE (GPIO, ADC, LEDs)
# ============================================================

print(yellow("Init PINs"))

# ADC: different pin depending on PA
analog_in = AnalogIn(board.GP27 if config.hasPa else board.GP26)

# LEDs
pwrLED = digitalio.DigitalInOut(board.GP9)
pwrLED.direction = digitalio.Direction.OUTPUT
pwrLED.value = True   # Power LED ON

gpsLED = digitalio.DigitalInOut(board.GP10)
gpsLED.direction = digitalio.Direction.OUTPUT
gpsLED.value = False

loraLED = digitalio.DigitalInOut(board.GP11)
loraLED.direction = digitalio.Direction.OUTPUT
loraLED.value = False

# PA enable pin
amp = digitalio.DigitalInOut(board.GP2)
amp.direction = digitalio.Direction.OUTPUT
amp.value = False

# I2C power pin
i2cPower = digitalio.DigitalInOut(board.GP3)
i2cPower.direction = digitalio.Direction.OUTPUT
i2cPower.value = False


# ============================================================
#       TELEMETRY SEQUENCE LOADING (NVM)
# ============================================================

print(yellow("Loading telemetry sequence…"))

nvm = NonVolatileMemory()
sequence = 0

try:
    stored = int(nvm.read_data())
    if 0 < stored <= 8191:
        sequence = stored
    else:
        nvm.save_data(0)
except Exception:
    nvm.save_data(0)

print(yellow("Sequence start at: " + str(sequence)))


# ============================================================
#       APRS ENCODER INITIALIZATION
# ============================================================

aprs = APRS()


# ============================================================
#       LORA MODULE INITIALIZATION
# ============================================================

print(yellow("Init LoRa"))

RADIO_FREQ_MHZ = 433.775

CS = digitalio.DigitalInOut(board.GP21)
RESET = digitalio.DigitalInOut(board.GP20)
spi = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)

try:
    rfm9x = adafruit_rfm9x.RFM9x(
        spi, CS, RESET,
        RADIO_FREQ_MHZ,
        baudrate=1000000
    )
    rfm9x.tx_power = config.power  # 5–23 dBm
    print(green("LoRa OK (freq = " + str(RADIO_FREQ_MHZ) + " MHz)"))
except Exception as e:
    print(red("LoRa INIT ERROR: " + str(e)))
    while True:
        w.feed()
        time.sleep(1)


# ============================================================
#       GPS INITIALIZATION
# ============================================================

print(yellow("Init GPS"))

uart = busio.UART(
    board.GP4, board.GP5,
    baudrate=9600,
    timeout=10,
    receiver_buffer_size=1024
)

gps = adafruit_gps.GPS(uart, debug=False)

# Set GPS to 1 Hz update
GPS_1HZ = bytes([
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
    0xE8, 0x03, 0x01, 0x00, 0x01, 0x00,
    0x01, 0x39
])
gps.send_command(GPS_1HZ)
time.sleep(0.1)

# Disable unwanted UBX messages (reduces memory, CPU load)
DISABLE_UBX = bytes([
    0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0xB9,
    0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0xC0,
])
gps.send_command(DISABLE_UBX)
time.sleep(0.1)


# ============================================================
#       TELEMETRY FRAME DEFINITIONS
# ============================================================

print(yellow("Init Telemetry"))

aprsData = [
    "PARM.Satelites",
    "UNIT.Nr",
    "EQNS.0,1,0"
]

# Add voltage telemetry
if config.voltage:
    for i, t in enumerate(aprsData):
        if t.startswith("PARM"):
            aprsData[i] += ",Battery"
        if t.startswith("UNIT"):
            aprsData[i] += ",Vdc"
        if t.startswith("EQNS"):
            aprsData[i] += ",0,0.01,0"

# I2C Sensors
shtc3 = False
bme680 = False

if config.i2cEnabled:
    print(yellow("Init I2C sensors…"))

    try:
        i2cPower.value = True
        time.sleep(2)
        i2c = busio.I2C(scl=board.GP7, sda=board.GP6)

        for dev in config.i2cDevices:
            if dev.lower() == "shtc3":
                import adafruit_shtc3
                i2c_shtc3 = adafruit_shtc3.SHTC3(i2c)
                shtc3 = True
                print(green("> SHTC3 OK"))
            elif dev.lower() == "bme680":
                import adafruit_bme680
                i2c_bme680 = adafruit_bme680.Adafruit_BME680_I2C(i2c)
                i2c_bme680.sea_level_pressure = 1015
                bme680 = True
                print(green("> BME680 OK"))

    except Exception as e:
        i2cPower.value = False
        print(red("I2C ERROR: " + str(e)))
        shtc3 = False
        bme680 = False


# ============================================================
#       TRACKING STATE INITIALIZATION
# ============================================================

print(yellow("Start Tracking"))

gps_lock = False
gps_blink = False

last_system_print = time.monotonic()
last_voltage_warning = 0

skip_first_bme680 = True

# Last real position (used for jitter freeze)
frozen_lat = None
frozen_lon = None

# ============================================================
#       MAIN LOOP — GPS UPDATE + SMARTBEACON DECISION ENGINE
# ============================================================

# Helpers for GPS freeze & movement classification
def freeze_position_if_stationary(lat, lon, speed):
    """Freezes position to last known true coordinate when not moving."""
    global frozen_lat, frozen_lon

    if speed < config.sb_stationarySpeed:
        # Freeze to last known
        if frozen_lat is not None and frozen_lon is not None:
            return frozen_lat, frozen_lon

        # If first freeze, store this
        frozen_lat = lat
        frozen_lon = lon
        return lat, lon

    # Moving → update freeze base position
    frozen_lat = lat
    frozen_lon = lon
    return lat, lon


while True:
    w.feed()

    # ------------------------------------------------------------
    #    GPS UPDATE
    # ------------------------------------------------------------
    try:
        gps.update()
    except MemoryError:
        print(yellow("GPS memory leak → REBOOTING"))
        microcontroller.reset()

    # ------------------------------------------------------------
    #    GPS FIX HANDLING + LED BLINK LOGIC
    # ------------------------------------------------------------
    if not gps.has_fix:
        if gps_blink:
            gpsLED.value = True
            gps_blink = False
        else:
            gpsLED.value = False
            gps_blink = True
            time.sleep(0.1)

        if config.fullDebug:
            print(yellow("No GPS FIX…"))
        continue

    # We now have a fix
    gpsLED.value = True

    # ------------------------------------------------------------
    #    REAL TIME CLOCK SYNC
    # ------------------------------------------------------------
    if year() == 1970 or year() < 2023:
        rtc.set_time_source(gps)
        the_rtc = rtc.RTC()
        print(yellow("RTC updated from GPS"))
        continue

    # ------------------------------------------------------------
    #    GPS POSITION + SPEED + HEADING
    # ------------------------------------------------------------
    lat = gps.latitude
    lon = gps.longitude
    alt = gps.altitude_m

    # Speed in km/h
    speed = -1
    heading = None

    if gps.track_angle_deg is not None:
        heading = gps.track_angle_deg
        speed = gps.speed_knots * 1.852  # kt → km/h

    # Freeze for stationary jitter removal
    lat, lon = freeze_position_if_stationary(lat, lon, speed)

    # ------------------------------------------------------------
    #    OPTIONAL DEBUG
    # ------------------------------------------------------------
    now = time.monotonic()
    if now - last_system_print >= 5:
        last_system_print = now

        print(purple(
            "FIX: LAT={:.6f}  LON={:.6f}  SPD={:.1f}km/h  HDG={}".format(
                lat, lon, speed, heading
            )
        ))

    # ============================================================
    #       SMARTBEACON DECISION
    # ============================================================

    sendBeacon = False

    if config.smartBeaconing:
        # SmartBeacon engine decides
        sendBeacon = SB.should_beacon(lat, lon, speed, heading)
    else:
        # Fallback legacy mode (rate + distance)
        dist_moved = distance(SB.last_lat, SB.last_lon, lat, lon)
        if SB.last_lat is None:
            sendBeacon = True
        elif dist_moved >= config.distance:
            sendBeacon = True
        elif now - SB.last_beacon_time >= config.rate:
            sendBeacon = True

    # ============================================================
    #       VOLTAGE MONITORING (INDEPENDENT OF MOVEMENT)
    # ============================================================

    if config.voltage and config.triggerVoltage:
        v = int(round(get_voltage(analog_in), 2) * 100)

        if 1000 <= v <= config.triggerVoltageLevel:
            # Only send voltage alerts at a limited rate
            if now - last_voltage_warning > config.triggerVoltageKeepalive:
                print(yellow("LOW VOLTAGE: {:.2f}V".format(v / 100)))
                last_voltage_warning = now
                # Actual APRS send done in Part 4
                pending_voltage_alert = v
            else:
                pending_voltage_alert = None
        else:
            pending_voltage_alert = None

    else:
        pending_voltage_alert = None

    # ============================================================
    #       CONTINUE OR SEND A BEACON?
    # ============================================================

    if not sendBeacon:
        time.sleep(0.05)
        continue

    # Mark beacon time for SmartBeacon
    SB.update_after_beacon(lat, lon)

    # SmartBeacon heading state update happens inside SB object
    # Now we proceed to build & send APRS packet (Part 4)
    break  # exit loop into Part 4 sending section

# ============================================================
#       PART 4: APRS PACKET CONSTRUCTION AND SENDING
# ============================================================

while True:  # Beacon sending loop
    w.feed()

    # ------------------------------------------------------------
    #   BUILD APRS TIMESTAMP
    # ------------------------------------------------------------
    ts = aprs.makeTimestamp(
        "z",
        gps.timestamp_utc.tm_mday,
        gps.timestamp_utc.tm_hour,
        gps.timestamp_utc.tm_min,
        gps.timestamp_utc.tm_sec,
    )

    # ------------------------------------------------------------
    #   POSITION PACKET
    # ------------------------------------------------------------
    aprs_pos = aprs.makePosition(
        lat, lon,
        speed,
        heading if heading is not None else -1,
        config.symbol
    )

    # ------------------------------------------------------------
    #   COMMENT + TELEMETRY
    # ------------------------------------------------------------
    telemetry_comment = config.comment

    # Telemetry sequence
    sequence = (sequence + 1) % 8192
    nvm.save_data(sequence)

    # Satellites / voltage / temp / hum
    telemetry_comment += "|" + base91_encode(sequence)

    # Satellites
    sats = gps.satellites if gps.satellites is not None else 0
    telemetry_comment += base91_encode(int(sats))

    # Voltage
    if config.voltage:
        bat_voltage = int(round(get_voltage(analog_in), 2) * 100)
        telemetry_comment += base91_encode(bat_voltage)

    # I2C sensors
    if shtc3:
        temp, hum = i2c_shtc3.measurements
        t_code = int((round(temp / 2, 2) + 25) * 100)
        h_code = int(round(hum, 0))
        telemetry_comment += base91_encode(t_code) + base91_encode(h_code)

    if bme680:
        global skip_first_bme680
        if skip_first_bme680:
            skip_first_bme680 = False
            _ = i2c_bme680.temperature
            _ = i2c_bme680.relative_humidity
        else:
            temp = i2c_bme680.temperature + config.bme680_tempOffset
            hum = i2c_bme680.relative_humidity
            t_code = int((round(temp / 2, 2) + 25) * 100)
            h_code = int(round(hum, 0))
            telemetry_comment += base91_encode(t_code) + base91_encode(h_code)

    telemetry_comment += "|"

    # Altitude
    if alt is not None:
        feet = int(alt * 3.2808399)
        telemetry_comment += "/A={:06d}".format(feet)

    # ------------------------------------------------------------
    #   FINAL APRS PACKET STRING
    # ------------------------------------------------------------
    full_message = "{}>APRFGT:@{}{}{}".format(
        config.callsign,
        ts,
        aprs_pos,
        telemetry_comment
    )

    print(purple("TX: " + full_message))

    # ------------------------------------------------------------
    #   SEND LORA PACKET
    # ------------------------------------------------------------
    try:
        loraLED.value = True

        if config.hasPa:
            amp.value = True
            time.sleep(0.25)

        rfm9x.send(
            w,
            b"<" +
            binascii.unhexlify("FF") +
            binascii.unhexlify("01") +
            bytes(full_message, "UTF-8")
        )

        if config.hasPa:
            time.sleep(0.1)
            amp.value = False

        loraLED.value = False

    except Exception as txerr:
        print(red("LoRa TX ERROR: " + str(txerr)))

    # Return to main SmartBeacon GPS loop
    break

# ============================================================
#      PART 5 — METADATA / VOLTAGE ALERTS / LOOP RETURN
# ============================================================

# We return here from the normal position-beacon send block
# and continue the overall tracking loop.

last_metadata_send = time.time() - 90000  # ensures metadata on first boot


while True:
    w.feed()

    # ------------------------------------------------------------
    # TELEMETRY METADATA (PARM / UNIT / EQNS)
    # Sent once per ~24 hours, or at boot
    # ------------------------------------------------------------
    now = time.time()

    if now - last_metadata_send >= 86400:
        last_metadata_send = now

        print(yellow("Sending daily Telemetry MetaDATA…"))

        if config.hasPa:
            amp.value = True
            time.sleep(0.25)

        for meta in aprsData:
            meta_msg = "{}>APRFGT::{:9}:{}".format(
                config.callsign, config.callsign, meta
            )

            print(purple("TX META: " + meta_msg))

            try:
                loraLED.value = True
                rfm9x.send(
                    w,
                    b"<" +
                    binascii.unhexlify("FF") +
                    binascii.unhexlify("01") +
                    bytes(meta_msg, "UTF-8")
                )
                loraLED.value = False
                time.sleep(0.15)

            except Exception as e:
                print(red("LoRa TX ERROR (meta): " + str(e)))

        if config.hasPa:
            time.sleep(0.1)
            amp.value = False

    # ------------------------------------------------------------
    # VOLTAGE ALERT MESSAGE (if pending)
    # ------------------------------------------------------------
    if config.voltage and config.triggerVoltage and ('pending_voltage_alert' in globals()):
        if pending_voltage_alert is not None:
            v = pending_voltage_alert / 100.0

            alert_msg = "{}>APRFGT::{:9}:Low Voltage ({:.2f}V) detected!".format(
                config.callsign,
                config.triggerVoltageCall,
                v
            )

            print(red("TX VOLT ALERT: " + alert_msg))

            try:
                if config.hasPa:
                    amp.value = True
                    time.sleep(0.25)

                loraLED.value = True
                rfm9x.send(
                    w,
                    b"<" +
                    binascii.unhexlify("FF") +
                    binascii.unhexlify("01") +
                    bytes(alert_msg, "UTF-8")
                )
                loraLED.value = False

                if config.hasPa:
                    time.sleep(0.1)
                    amp.value = False

            except Exception as e:
                print(red("LoRa TX ERROR (voltage alert): " + str(e)))

        # Reset pending alert flag
        pending_voltage_alert = None

    # ------------------------------------------------------------
    # RETURN TO GPS / SMARTBEACON LOOP
    # ------------------------------------------------------------
    break  # exit back to the GPS → SmartBeacon decision loop

# End PART 5
# Main loop continues back at the GPS update SmartBeacon block in PART 3
