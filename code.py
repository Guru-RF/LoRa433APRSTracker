# ============================================================
# RF.Guru LoRa APRS Tracker (SmartBeaconing Edition)
# - Fixes: compile/runtime issues + stationary GPS jitter (deadband + hysteresis)
# - Runs continuously
# - Uses config presets: sb_fastRate/sb_slowRate/sb_fastSpeed/sb_slowSpeed/...
# - Voltage alert + daily telemetry metadata supported
# ============================================================

import binascii
import time
from math import atan2, cos, radians, sin, sqrt

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

w.timeout = 5
w.mode = WatchDogMode.RESET
w.feed()

# Optional serial (kept for compatibility)
serial = usb_cdc.data

time.sleep(3)
w.feed()

VERSION = "RF.Guru_LoRa433APRSTracker 1.1-SB"

# ============================================================
#       HELPERS
# ============================================================

def _format_datetime(dt):
    return "{:02}/{:02}/{} {:02}:{:02}:{:02}".format(
        dt.tm_mon, dt.tm_mday, dt.tm_year, dt.tm_hour, dt.tm_min, dt.tm_sec
    )

def purple(msg):
    stamp = _format_datetime(time.localtime())
    return "\x1b[38;5;104m[" + str(stamp) + "] " + msg + "\x1b[0m"

def yellow(msg):
    return "\x1b[38;5;220m" + msg + "\x1b[0m"

def red(msg):
    return "\x1b[1;5;31m" + msg + "\x1b[0m"

def green(msg):
    return "\x1b[1;5;32m" + msg + "\x1b[0m"

def year():
    return int(time.localtime().tm_year)

def distance(lat1, lon1, lat2, lon2):
    """Distance in meters (haversine)."""
    if lat1 is None or lon1 is None or lat2 is None or lon2 is None:
        return 999999
    radius_km = 6371
    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = sin(dlat / 2) ** 2 + cos(radians(lat1)) * cos(radians(lat2)) * sin(dlon / 2) ** 2
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return int(radius_km * c * 1000)

def get_voltage(pin):
    # Keep your original scaling
    if config.hasPa:
        return ((pin.value * 3.3) / 65536) + 10.6 + 1.4
    return ((pin.value * 3.3) / 65536) * 2

def base91_encode(n):
    """
    Encode a non-negative integer to APRS Base91 with at least 2 chars.
    """
    n = int(n)
    if n < 0:
        raise ValueError("base91_encode expects non-negative integer")

    out = []
    while True:
        out.append(chr(33 + (n % 91)))
        n //= 91
        if n == 0:
            break

    if len(out) == 1:
        out.append("!")  # pad to 2 chars

    return "".join(reversed(out))

# ============================================================
#        CONFIG VALIDATION (matches your SmartBeacon config)
# ============================================================

def _is_bool(x):
    return isinstance(x, bool)

try:
    if not _is_bool(config.fullDebug):
        raise AttributeError("fullDebug must be True/False")

    if not (5 <= int(config.power) <= 23):
        raise AttributeError("power must be between 5 and 23 dBm")

    if not _is_bool(config.hasPa):
        raise AttributeError("hasPa must be True/False")

    if not isinstance(config.callsign, str) or not config.callsign.strip():
        raise AttributeError("callsign must be a non-empty string")

    if not isinstance(config.symbol, str) or len(config.symbol) < 2:
        raise AttributeError("symbol must be a string like 'L>' (2 chars)")

    if not isinstance(config.comment, str):
        raise AttributeError("comment must be a string")

    # Legacy compatibility (only used if smartBeaconing=False)
    if hasattr(config, "smartBeaconing") and (not config.smartBeaconing):
        if not (10 <= int(config.rate) <= 60):
            raise AttributeError("rate must be 10–60 sec")
        if not (300 <= int(config.keepalive) <= 3600):
            raise AttributeError("keepalive must be 300–3600 sec")
        if not (25 <= int(config.distance) <= 1000):
            raise AttributeError("distance must be 25–1000 m")

    if not _is_bool(config.voltage):
        raise AttributeError("voltage must be True/False")

    if not _is_bool(config.triggerVoltage):
        raise AttributeError("triggerVoltage must be True/False")

    if not (250 <= int(config.triggerVoltageLevel) <= 1280):
        raise AttributeError("triggerVoltageLevel must be 250–1280 (V*100)")

    if not isinstance(config.triggerVoltageCall, str) or not config.triggerVoltageCall.strip():
        raise AttributeError("triggerVoltageCall must be a string")

    if not (3600 <= int(config.triggerVoltageKeepalive) <= 14400):
        raise AttributeError("triggerVoltageKeepalive must be 3600–14400 sec")

    if not _is_bool(config.i2cEnabled):
        raise AttributeError("i2cEnabled must be True/False")

    if type(config.i2cDevices) not in (list, tuple):
        raise AttributeError("i2cDevices must be list or tuple")

    if not (0 <= int(config.bme680_tempOffset) <= 99):
        raise AttributeError("bme680_tempOffset must be 0–99")

    # SmartBeacon required keys
    if not _is_bool(config.smartBeaconing):
        raise AttributeError("smartBeaconing must be True/False")

    if not (5 <= int(config.sb_fastRate) <= 120):
        raise AttributeError("sb_fastRate must be 5–120 sec")

    if not (30 <= int(config.sb_slowRate) <= 900):
        raise AttributeError("sb_slowRate must be 30–900 sec")

    # Allow sub-1.0 for bike/hiker
    if not (0 <= float(config.sb_stationarySpeed) <= 10):
        raise AttributeError("sb_stationarySpeed must be 0–10 km/h")

    if not (0 <= float(config.sb_slowSpeed) < float(config.sb_fastSpeed) <= 200):
        raise AttributeError("sb_slowSpeed must be < sb_fastSpeed (0–200 km/h)")

    # Stationary jitter filter (distance + hysteresis)
    if not hasattr(config, "sb_stationaryDistance"):
        raise AttributeError("sb_stationaryDistance missing (meters)")
    if not (1 <= int(config.sb_stationaryDistance) <= 500):
        raise AttributeError("sb_stationaryDistance must be 1–500 meters")

    if not hasattr(config, "sb_stationaryExitCount"):
        raise AttributeError("sb_stationaryExitCount missing (1..10)")
    if not (1 <= int(config.sb_stationaryExitCount) <= 10):
        raise AttributeError("sb_stationaryExitCount must be 1–10")

    if not (5 <= int(config.sb_turnThreshold) <= 90):
        raise AttributeError("sb_turnThreshold must be 5–90 degrees")

    if not (0 <= float(config.sb_turnSlope) <= 20):
        raise AttributeError("sb_turnSlope must be 0–20 deg/sec")

    if not _is_bool(config.sb_headingFilter):
        raise AttributeError("sb_headingFilter must be True/False")

except AttributeError as e:
    print(red("CONFIG ERROR: " + str(e)))
    while True:
        w.feed()
        time.sleep(1)

# Normalize
config.callsign = config.callsign.upper().strip()
config.triggerVoltageCall = config.triggerVoltageCall.upper().strip()

# ============================================================
#        SMARTBEACONING ENGINE
# ============================================================

class SmartBeacon:
    """
    SmartBeaconing features:
      - fast/slow rate selection based on speed (uses slowSpeed..fastSpeed)
      - optional corner pegging (turnThreshold/turnSlope)
      - stationary interval (slowRate)
      - heading filter that correctly handles 359° <-> 0° wrap
    """

    def __init__(self):
        self.last_beacon_time = time.monotonic() - 9999  # allow immediate first beacon
        self.last_lat = None
        self.last_lon = None

        self.last_heading = None
        self.last_heading_time = None
        self.filtered_heading = None

    @staticmethod
    def _angle_diff(a, b):
        diff = abs(a - b)
        if diff > 180:
            diff = 360 - diff
        return diff

    def filter_heading(self, heading):
        if heading is None:
            return None
        if not config.sb_headingFilter:
            return heading

        if self.filtered_heading is None:
            self.filtered_heading = heading
            return heading

        delta = ((heading - self.filtered_heading + 540) % 360) - 180
        alpha = 0.3
        self.filtered_heading = (self.filtered_heading + alpha * delta) % 360
        return self.filtered_heading

    def heading_change(self, heading, now):
        if heading is None:
            return 0, 0

        if self.last_heading is None or self.last_heading_time is None:
            self.last_heading = heading
            self.last_heading_time = now
            return 0, 0

        diff = self._angle_diff(heading, self.last_heading)
        dt = now - self.last_heading_time
        slope = diff / max(dt, 0.1)

        self.last_heading = heading
        self.last_heading_time = now

        return diff, slope

    def _interval_for_speed(self, speed_kmh):
        slow_spd = float(config.sb_slowSpeed)
        fast_spd = float(config.sb_fastSpeed)
        slow_rate = float(config.sb_slowRate)
        fast_rate = float(config.sb_fastRate)

        if speed_kmh <= slow_spd:
            return slow_rate
        if speed_kmh >= fast_spd:
            return fast_rate

        frac = (speed_kmh - slow_spd) / (fast_spd - slow_spd)
        return slow_rate - frac * (slow_rate - fast_rate)

    def should_beacon(self, lat, lon, speed_kmh, heading):
        now = time.monotonic()
        dt_beacon = now - self.last_beacon_time

        # First beacon after boot/fix
        if self.last_lat is None or self.last_lon is None:
            fh = self.filter_heading(heading)
            _ = self.heading_change(fh, now)
            return True

        if speed_kmh is None:
            speed_kmh = 0.0

        fh = self.filter_heading(heading)
        diff, slope = self.heading_change(fh, now)

        # Stationary / near-stationary => slowRate
        if speed_kmh < float(config.sb_stationarySpeed):
            return dt_beacon >= float(config.sb_slowRate)

        interval = self._interval_for_speed(speed_kmh)

        # Timed beacon
        if dt_beacon >= interval:
            return True

        # Corner pegging (don't exceed fastRate)
        if (speed_kmh >= float(config.sb_slowSpeed)) and (fh is not None) and (dt_beacon >= float(config.sb_fastRate)):
            if diff >= float(config.sb_turnThreshold):
                return True
            if slope >= float(config.sb_turnSlope):
                return True

        return False

    def update_after_beacon(self, lat, lon):
        self.last_beacon_time = time.monotonic()
        self.last_lat = lat
        self.last_lon = lon


SB = SmartBeacon()

print(red(" -- Tracker Booted: " + config.callsign + " -=- " + VERSION))
print(yellow("SmartBeaconing: " + str(config.smartBeaconing)))

# ============================================================
#       INITIALIZE HARDWARE (GPIO, ADC, LEDs)
# ============================================================

print(yellow("Init PINs"))

analog_in = AnalogIn(board.GP27 if config.hasPa else board.GP26)

pwrLED = digitalio.DigitalInOut(board.GP9)
pwrLED.direction = digitalio.Direction.OUTPUT
pwrLED.value = True

gpsLED = digitalio.DigitalInOut(board.GP10)
gpsLED.direction = digitalio.Direction.OUTPUT
gpsLED.value = False

loraLED = digitalio.DigitalInOut(board.GP11)
loraLED.direction = digitalio.Direction.OUTPUT
loraLED.value = False

amp = digitalio.DigitalInOut(board.GP2)
amp.direction = digitalio.Direction.OUTPUT
amp.value = False

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
#       APRS ENCODER + LORA INIT
# ============================================================

aprs = APRS()

print(yellow("Init LoRa"))
RADIO_FREQ_MHZ = float(getattr(config, "loraFrequency", 433.775))

CS = digitalio.DigitalInOut(board.GP21)
RESET = digitalio.DigitalInOut(board.GP20)
spi = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)

try:
    rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ, baudrate=1000000)
    rfm9x.tx_power = int(config.power)
    print(green("LoRa OK (freq = " + str(RADIO_FREQ_MHZ) + " MHz)"))
except Exception as e:
    print(red("LoRa INIT ERROR: " + str(e)))
    while True:
        w.feed()
        time.sleep(1)

LORA_HEADER = b"<\xff\x01"

def lora_send_text(text):
    w.feed()
    loraLED.value = True

    if config.hasPa:
        amp.value = True
        time.sleep(0.25)

    try:
        rfm9x.send(w, LORA_HEADER + bytes(text, "UTF-8"))
    finally:
        if config.hasPa:
            time.sleep(0.1)
            amp.value = False
        loraLED.value = False
        w.feed()

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

GPS_1HZ = bytes([
    0xB5, 0x62, 0x06, 0x08, 0x06, 0x00,
    0xE8, 0x03, 0x01, 0x00, 0x01, 0x00,
    0x01, 0x39
])
gps.send_command(GPS_1HZ)
time.sleep(0.1)

DISABLE_UBX = bytes([
    0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x12,0xB9,
    0xB5,0x62,0x06,0x01,0x08,0x00,0x01,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x13,0xC0,
])
gps.send_command(DISABLE_UBX)
time.sleep(0.1)

# ============================================================
#       TELEMETRY METADATA DEFINITIONS
# ============================================================

print(yellow("Init Telemetry"))

aprsData = [
    "PARM.Satelites",
    "UNIT.Nr",
    "EQNS.0,1,0"
]

if config.voltage:
    for i, t in enumerate(aprsData):
        if t.startswith("PARM"):
            aprsData[i] += ",Battery"
        elif t.startswith("UNIT"):
            aprsData[i] += ",Vdc"
        elif t.startswith("EQNS"):
            aprsData[i] += ",0,0.01,0"

shtc3 = False
bme680 = False
i2c_shtc3 = None
i2c_bme680 = None

if config.i2cEnabled:
    print(yellow("Init I2C sensors…"))
    try:
        i2cPower.value = True
        w.feed()
        time.sleep(2)
        w.feed()

        i2c = busio.I2C(scl=board.GP7, sda=board.GP6)

        for dev in config.i2cDevices:
            dev_l = str(dev).lower()

            if dev_l == "shtc3":
                import adafruit_shtc3
                i2c_shtc3 = adafruit_shtc3.SHTC3(i2c)
                shtc3 = True
                print(green("> SHTC3 OK"))

                for i, t in enumerate(aprsData):
                    if t.startswith("PARM"):
                        aprsData[i] += ",Temp,Hum"
                    elif t.startswith("UNIT"):
                        aprsData[i] += ",deg.C,%"
                    elif t.startswith("EQNS"):
                        aprsData[i] += ",0,0.02,-50,0,1,0"
                break

            if dev_l == "bme680":
                import adafruit_bme680
                i2c_bme680 = adafruit_bme680.Adafruit_BME680_I2C(i2c)
                i2c_bme680.sea_level_pressure = 1015
                bme680 = True
                print(green("> BME680 OK"))

                for i, t in enumerate(aprsData):
                    if t.startswith("PARM"):
                        aprsData[i] += ",Temp,Hum"
                    elif t.startswith("UNIT"):
                        aprsData[i] += ",deg.C,%"
                    elif t.startswith("EQNS"):
                        aprsData[i] += ",0,0.02,-50,0,1,0"
                break

    except Exception as e:
        i2cPower.value = False
        print(red("I2C ERROR: " + str(e)))
        shtc3 = False
        bme680 = False
        i2c_shtc3 = None
        i2c_bme680 = None

# ============================================================
#       TRACKING STATE
# ============================================================

print(yellow("Start Tracking"))

gps_blink = False
last_debug_print = time.monotonic()

last_voltage_warning = 0
pending_voltage_alert = None

skip_first_bme680 = True

# Stationary jitter lock (distance-based + hysteresis)
anchor_lat = None
anchor_lon = None
stationary_mode = False
stationary_exit_hits = 0

last_metadata_send = time.monotonic() - 90000  # ensure metadata soon after boot

def stabilize_position(raw_lat, raw_lon, speed_kmh):
    """
    Reduce stationary GPS jitter:
      - When stationary_mode is active, output the anchor position.
      - Exit stationary_mode only when the raw position is >= sb_stationaryDistance
        for sb_stationaryExitCount consecutive fixes (filters GPS spikes).

    Returns: (lat, lon, stationary_mode, moved_m, started_moving)
    """
    global anchor_lat, anchor_lon, stationary_mode, stationary_exit_hits

    started_moving = False

    if raw_lat is None or raw_lon is None:
        return raw_lat, raw_lon, stationary_mode, 0, started_moving

    if anchor_lat is None or anchor_lon is None:
        anchor_lat, anchor_lon = raw_lat, raw_lon
        if speed_kmh is None:
            speed_kmh = 0.0
        stationary_mode = (speed_kmh < float(config.sb_stationarySpeed))
        stationary_exit_hits = 0
        return raw_lat, raw_lon, stationary_mode, 0, started_moving

    moved_m = distance(anchor_lat, anchor_lon, raw_lat, raw_lon)
    thresh = int(config.sb_stationaryDistance)
    need = int(config.sb_stationaryExitCount)

    if stationary_mode:
        if moved_m >= thresh:
            stationary_exit_hits += 1
            if stationary_exit_hits >= need:
                stationary_mode = False
                stationary_exit_hits = 0
                anchor_lat, anchor_lon = raw_lat, raw_lon
                started_moving = True
                return raw_lat, raw_lon, stationary_mode, moved_m, started_moving

            # Still stationary until we confirm movement
            return anchor_lat, anchor_lon, stationary_mode, moved_m, started_moving

        stationary_exit_hits = 0
        return anchor_lat, anchor_lon, stationary_mode, moved_m, started_moving

    # Not stationary -> follow GPS
    anchor_lat, anchor_lon = raw_lat, raw_lon
    stationary_exit_hits = 0

    if speed_kmh is None:
        speed_kmh = 0.0
    if speed_kmh < float(config.sb_stationarySpeed):
        stationary_mode = True

    return raw_lat, raw_lon, stationary_mode, moved_m, started_moving

def build_and_send_metadata():
    global last_metadata_send
    last_metadata_send = time.monotonic()

    print(yellow("Sending Telemetry MetaDATA…"))
    for meta in aprsData:
        w.feed()
        meta_msg = "{}>APRFGT::{:9}:{}".format(config.callsign, config.callsign, meta)
        print(purple("TX META: " + meta_msg))
        try:
            lora_send_text(meta_msg)
        except Exception as e:
            print(red("LoRa TX ERROR (meta): " + str(e)))
        time.sleep(0.15)

def send_voltage_alert(v_x100):
    v = v_x100 / 100.0
    msg = "{}>APRFGT::{:9}:Low Voltage ({:.2f}V) detected!".format(
        config.callsign,
        config.triggerVoltageCall,
        v
    )
    print(red("TX VOLT ALERT: " + msg))
    try:
        lora_send_text(msg)
    except Exception as e:
        print(red("LoRa TX ERROR (voltage alert): " + str(e)))

# ============================================================
#       MAIN LOOP
# ============================================================

try:
    while True:
        w.feed()

        # GPS update
        try:
            gps.update()
        except MemoryError:
            print(yellow("GPS memory leak → REBOOTING"))
            time.sleep(0.5)
            microcontroller.reset()

        # No fix -> blink
        if not gps.has_fix:
            gpsLED.value = gps_blink
            gps_blink = not gps_blink
            if not gpsLED.value:
                time.sleep(0.1)
            if config.fullDebug:
                print(yellow("No GPS FIX…"))
            continue

        gpsLED.value = True

        # Sync RTC from GPS if needed
        if year() == 1970 or year() < 2023:
            rtc.set_time_source(gps)
            _ = rtc.RTC()
            if config.fullDebug:
                print(yellow("RTC updated from GPS"))
            continue

        if gps.timestamp_utc is None:
            if config.fullDebug:
                print(yellow("GPS timestamp not ready…"))
            time.sleep(0.05)
            continue

        # Read GPS data
        raw_lat = gps.latitude
        raw_lon = gps.longitude
        alt_m = gps.altitude_m

        heading = gps.track_angle_deg

        speed_kmh = None
        if gps.speed_knots is not None:
            speed_kmh = gps.speed_knots * 1.852

        # Stabilize stationary jitter
        lat, lon, stationary_mode, jitter_m, started_moving = stabilize_position(
            raw_lat, raw_lon, speed_kmh
        )

        # Periodic debug print
        now_mono = time.monotonic()
        if now_mono - last_debug_print >= 5:
            last_debug_print = now_mono
            print(purple(
                "FIX: LAT={:.6f} LON={:.6f} SPD={}km/h HDG={} ST={} JIT={}m".format(
                    lat, lon,
                    "{:.1f}".format(speed_kmh) if speed_kmh is not None else "NA",
                    heading if heading is not None else "NA",
                    "Y" if stationary_mode else "N",
                    jitter_m
                )
            ))

        # SmartBeacon decision
        send_beacon = False
        if config.smartBeaconing:
            # When stationary_mode is active, force speed=0 so SmartBeacon doesn't "move" from speed noise
            sb_speed = 0.0 if stationary_mode else (speed_kmh if speed_kmh is not None else 0.0)

            # started_moving forces an immediate beacon when we unlock from stationary
            send_beacon = started_moving or SB.should_beacon(lat, lon, sb_speed, heading)
        else:
            # Legacy fallback
            dt = now_mono - SB.last_beacon_time
            moved = distance(SB.last_lat, SB.last_lon, lat, lon)
            if SB.last_lat is None:
                send_beacon = True
            elif moved >= int(config.distance):
                send_beacon = True
            elif dt >= int(config.rate):
                send_beacon = True

        # Voltage monitoring (independent)
        if config.voltage and config.triggerVoltage:
            v_x100 = int(round(get_voltage(analog_in), 2) * 100)
            if 1000 <= v_x100 <= int(config.triggerVoltageLevel):
                if (now_mono - last_voltage_warning) > int(config.triggerVoltageKeepalive):
                    print(yellow("LOW VOLTAGE: {:.2f}V".format(v_x100 / 100.0)))
                    last_voltage_warning = now_mono
                    pending_voltage_alert = v_x100
            else:
                pending_voltage_alert = None
        else:
            pending_voltage_alert = None

        # Metadata timer
        metadata_due = (now_mono - last_metadata_send) >= 86400

        # Idle
        if (not send_beacon) and (pending_voltage_alert is None) and (not metadata_due):
            time.sleep(0.05)
            continue

        # Send metadata even if not beaconing
        if metadata_due and (not send_beacon) and (pending_voltage_alert is None):
            build_and_send_metadata()
            time.sleep(0.05)
            continue

        # ------------------------------------------------------------
        # SEND POSITION BEACON
        # ------------------------------------------------------------
        if send_beacon:
            ts = aprs.makeTimestamp(
                "z",
                gps.timestamp_utc.tm_mday,
                gps.timestamp_utc.tm_hour,
                gps.timestamp_utc.tm_min,
                gps.timestamp_utc.tm_sec,
            )

            spd_for_aprs = speed_kmh if speed_kmh is not None else -1
            hdg_for_aprs = heading if heading is not None else -1

            aprs_pos = aprs.makePosition(lat, lon, spd_for_aprs, hdg_for_aprs, config.symbol)

            # Telemetry sequence
            sequence = (sequence + 1) % 8192
            try:
                nvm.save_data(sequence)
            except Exception:
                pass

            comment = config.comment
            comment += "|" + base91_encode(sequence)

            sats = gps.satellites if gps.satellites is not None else 0
            comment += base91_encode(int(sats))

            if config.voltage:
                bat_x100 = int(round(get_voltage(analog_in), 2) * 100)
                comment += base91_encode(bat_x100)

            # Sensor telemetry (Temp/Hum)
            if shtc3 and (i2c_shtc3 is not None):
                t_c, rh = i2c_shtc3.measurements
                t_code = int((round(t_c / 2.0, 2) + 25) * 100)
                h_code = int(round(rh, 0))
                comment += base91_encode(t_code) + base91_encode(h_code)

            elif bme680 and (i2c_bme680 is not None):
                if skip_first_bme680:
                    skip_first_bme680 = False
                    _ = i2c_bme680.temperature
                    _ = i2c_bme680.relative_humidity
                else:
                    t_c = i2c_bme680.temperature + float(config.bme680_tempOffset)
                    rh = i2c_bme680.relative_humidity
                    t_code = int((round(t_c / 2.0, 2) + 25) * 100)
                    h_code = int(round(rh, 0))
                    comment += base91_encode(t_code) + base91_encode(h_code)

            comment += "|"

            if alt_m is not None:
                feet = int(alt_m * 3.2808399)
                comment += "/A={:06d}".format(feet)

            frame = "{}>APRFGT:@{}{}{}".format(config.callsign, ts, aprs_pos, comment)
            print(purple("TX: " + frame))

            try:
                lora_send_text(frame)
                SB.update_after_beacon(lat, lon)
            except Exception as e:
                print(red("LoRa TX ERROR: " + str(e)))

        # ------------------------------------------------------------
        # Send metadata if due
        # ------------------------------------------------------------
        if metadata_due:
            build_and_send_metadata()

        # ------------------------------------------------------------
        # Voltage alert if pending
        # ------------------------------------------------------------
        if pending_voltage_alert is not None:
            send_voltage_alert(pending_voltage_alert)
            pending_voltage_alert = None

        time.sleep(0.05)

except Exception as fatal:
    print(red("FATAL ERROR → rebooting: " + str(fatal)))
    time.sleep(1)
    microcontroller.reset()
