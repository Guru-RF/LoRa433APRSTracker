import binascii
import os
import time
from math import atan2, ceil, cos, log, radians, sin, sqrt

import adafruit_gps
import adafruit_rfm9x
import board
import busio
import config
import digitalio
import microcontroller
import rtc
import supervisor
import usb_cdc
from analogio import AnalogIn
from APRS import APRS
from microcontroller import watchdog as w
from watchdog import WatchDogMode

# stop autoreloading
supervisor.runtime.autoreload = False

# configure watchdog
w.timeout = 5
w.mode = WatchDogMode.RESET
w.feed()


def cleanup():
    try:
        os.unlink("/check")
        os.unlink("/ro")
        return True
    except OSError:
        return True


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
    return "\x1b[1;5;31m -- " + data + "\x1b[0m"


# geometry distance calculator in meters
def distance(lat1, lon1, lat2, lon2):
    if lat1 is None:
        return 999999
    radius = 6371  # km

    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = sin(dlat / 2) * sin(dlat / 2) + cos(radians(lat1)) * cos(radians(lat2)) * sin(
        dlon / 2
    ) * sin(dlon / 2)
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return int(round(radius * c * 1000, 0))


# voltage meter (12v/4v)
def get_voltage(pin):
    if config.hasPa is True:
        return ((pin.value * 3.3) / 65536) + 10.6 + 0.7
    else:
        return ((pin.value * 3.3) / 65536) * 2


# telemetry encoder
def base91_encode(number):
    text = []

    if number < 0:
        raise ValueError("Expected number to be positive integer")
    elif number > 0:
        max_n = ceil(log(number) / log(91))

        for n in range(int(max_n), -1, -1):
            quotient, number = divmod(number, 91**n)
            text.append(chr(33 + quotient))

    text = "".join(text).lstrip("!")
    if len(text) == 1:
        text = "!" + text

    return text


# serial input
serial = usb_cdc.data

# wait for console()
time.sleep(3)
w.feed()

# our version
VERSION = "RF.Guru_LoRaAPRStracker 0.1"

# read telemetry sequence (sleep when in RO)
sequence = 0
try:
    with open("/check", "w") as f:
        f.write("ok")
        f.close()
    with open("/sequence", "r") as f:
        sequence = int(f.read())
        f.close()
    cleanup()
except:
    w.feed()
    time.sleep(1)
    print(
        yellow(
            "In configure mode, usb disk is present for editing, remove file ro for normal mode !!"
        )
    )
    print(yellow("Sleep until hard reset (unplug and replug power)"))
    serial.write(
        str.encode(
            "In configure mode, usb disk is present for editing, remove file ro for normal mode !!\r\n\r\nSleep until hard reset (unplug and replug power)\r\n\r\n"
        )
    )
    while True:
        w.feed()

print(yellow("Use other serial port for firemware and configuration mode !"))
if serial is not None:
    if serial.connected:
        serial.write(
            str.encode(
                "Press f key for rp2040 firmware upgrade (enables usb firmware disk drive)\r\n\r\n"
            )
        )
        serial.write(
            str.encode(
                "Press any other key for configuration mode (enables usb disk drive)\r\n\r\n"
            )
        )
t_end = time.time() + 5
while time.time() < t_end:
    w.feed()
    if serial.in_waiting > 0:
        letter = serial.read().decode("utf-8")
        if letter == "f":
            print(
                yellow(
                    "Reboot into firmware mode, usb firmware disk will popup after reboot ..."
                )
            )
            serial.write(
                str.encode(
                    "Reboot into firmware mode, usb firmware disk will popup after reboot ...\r\n"
                )
            )
            time.sleep(1)
            microcontroller.on_next_reset(microcontroller.RunMode.UF2)
            microcontroller.reset()
        with open("/ro", "w") as f:
            f.write("ok")
            f.close()
        print()
        print(
            yellow(
                "Reboot into configuration mode, usb disk will popup after reboot ..."
            )
        )
        serial.write(
            str.encode(
                "Reboot into configuration mode, usb disk will popup after reboot ...\r\n"
            )
        )
        time.sleep(1)
        microcontroller.reset()

if config.callsign == "":
    with open("/ro", "w") as f:
        f.write("ok")
        f.close()
    print()
    print(
        yellow(
            "No callsign defined in config file reboot into configuration mode, usb disk will popup after reboot ..."
        )
    )
    serial.write(
        str.encode(
            "Reboot into configuration mode, usb disk will popup after reboot ...\r\n"
        )
    )
    time.sleep(1)
    microcontroller.reset()


def _format_datetime(datetime):
    return "{:02}/{:02}/{} {:02}:{:02}:{:02}".format(
        datetime.tm_mon,
        datetime.tm_mday,
        datetime.tm_year,
        datetime.tm_hour,
        datetime.tm_min,
        datetime.tm_sec,
    )


print(red(config.callsign + " -=- " + VERSION))

print(yellow("Init PINs"))

# voltage adc
analog_in = AnalogIn(board.GP27)
if config.hasPa is False:
    analog_in = AnalogIn(board.GP26)

# configure LEDs
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

try:
    # APRS encoder
    aprs = APRS()

    print(yellow("Init LoRa"))

    # LoRa APRS frequency
    RADIO_FREQ_MHZ = 433.775
    CS = digitalio.DigitalInOut(board.GP21)
    RESET = digitalio.DigitalInOut(board.GP20)
    spi = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)

    # Lora Module
    rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ, baudrate=1000000)
    rfm9x.tx_power = config.power  # 5 min 23 max

    print(yellow("Init GPS"))
    # GPS Module (uart)
    uart = busio.UART(
        board.GP4, board.GP5, baudrate=9600, timeout=10, receiver_buffer_size=1024
    )
    gps = adafruit_gps.GPS(uart, debug=False)

    # Set GPS speed to 1HZ
    Speed = bytes(
        [
            0xB5,
            0x62,
            0x06,
            0x08,
            0x06,
            0x00,
            0xE8,
            0x03,
            0x01,
            0x00,
            0x01,
            0x00,
            0x01,
            0x39,  # 1Hz
            # 0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A 5hz
        ]
    )
    gps.send_command(Speed)
    time.sleep(0.1)

    ## Disable UBX data
    Disable_UBX = bytes(
        [
            0xB5,
            0x62,
            0x06,
            0x01,
            0x08,
            0x00,
            0x01,
            0x02,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x12,
            0xB9,  # NAV-POSLLH
            0xB5,
            0x62,
            0x06,
            0x01,
            0x08,
            0x00,
            0x01,
            0x03,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x13,
            0xC0,  # NAV-STATUS
            0xB5,
            0x62,
            0x06,
            0x01,
            0x08,
            0x00,
            0x01,
            0x03,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x00,
            0x13,
            0xC0,  # NAV-STATUS
        ]
    )
    gps.send_command(Disable_UBX)
    time.sleep(0.1)

    print(yellow("Init Telemetry"))

    # default telemetry
    aprsData = ["PARM.Satelites", "UNIT.Nr", "EQNS.0,1,0"]

    # add voltage meter if present
    if config.voltage is True:
        for index, item in enumerate(aprsData):
            if item.startswith("PARM"):
                aprsData[index] = aprsData[index] + ",Battery"
            if item.startswith("UNIT"):
                aprsData[index] = aprsData[index] + ",Vdc"
            if item.startswith("EQNS"):
                aprsData[index] = aprsData[index] + ",0,0.01,0"
    else:
        for index, item in enumerate(aprsData):
            if item.startswith("PARM"):
                aprsData[index] = aprsData[index] + ","
            if item.startswith("UNIT"):
                aprsData[index] = aprsData[index] + ","
            if item.startswith("EQNS"):
                aprsData[index] = aprsData[index] + ",0,0,0"

    # i2c
    shtc3 = False
    bme680 = False
    # disable extra telemetry fields if no i2c
    if config.i2cEnabled is False:
        for index, item in enumerate(aprsData):
            if item.startswith("PARM"):
                aprsData[index] = aprsData[index] + ",,"
            if item.startswith("UNIT"):
                aprsData[index] = aprsData[index] + ",,"
            if item.startswith("EQNS"):
                aprsData[index] = aprsData[index] + ",0,0,0,0,0,0"
    else:
        print(yellow("Init i2c Modules"))
        # i2c modules

        try:
            # power on i2c
            i2cPower.value = True
            time.sleep(1)
            i2c = busio.I2C(scl=board.GP7, sda=board.GP6)
            for idex, item in enumerate(config.i2cDevices):
                if item.lower() == "shtc3":
                    for index, item in enumerate(aprsData):
                        if item.startswith("PARM"):
                            aprsData[index] = aprsData[index] + ",Temperature,Humidity"
                        if item.startswith("UNIT"):
                            aprsData[index] = aprsData[index] + ",deg.C,%"
                        if item.startswith("EQNS"):
                            aprsData[index] = aprsData[index] + ",0,0.02,-50,0,1,0"
                    import adafruit_shtc3

                    i2c_shtc3 = adafruit_shtc3.SHTC3(i2c)
                    shtc3 = True
                    print(yellow(">shtc loaded"))
                if item.lower() == "bme680":
                    for index, item in enumerate(aprsData):
                        if item.startswith("PARM"):
                            aprsData[index] = aprsData[index] + ",Temperature,Humidity"
                        if item.startswith("UNIT"):
                            aprsData[index] = aprsData[index] + ",deg.C,%"
                        if item.startswith("EQNS"):
                            aprsData[index] = aprsData[index] + ",0,0.02,-50,0,1,0"
                    import adafruit_bme680

                    i2c_bme680 = adafruit_bme680.Adafruit_BME680_I2C(i2c)
                    i2c_bme680.sea_level_pressure = 1015
                    bme680 = True
                    print(yellow(">bme680 loaded"))
        except Exception as error:
            i2cPower.value = False
            print("I2C err reloading: ", error)
            time.sleep(1)
            microcontroller.reset()

    print(yellow("Send Telemetry MetaDATA"))
    # send telemetry metadata once
    for data in aprsData:
        message = "{}>APRFGT::{}:{}".format(config.callsign, config.callsign, data)
        loraLED.value = True
        if config.hasPa is True:
            amp.value = True
            time.sleep(0.3)
        print(yellow("LoRa send message: " + message))
        rfm9x.send(
            w,
            bytes("{}".format("<"), "UTF-8")
            + binascii.unhexlify("FF")
            + binascii.unhexlify("01")
            + bytes("{}".format(message), "UTF-8"),
        )
        if config.hasPa is True:
            time.sleep(0.1)
            amp.value = False
        loraLED.value = False
        time.sleep(0.5)

    print(yellow("Start Tracking"))

    # start tracking
    last_print = time.monotonic()
    last_lat = None
    last_lon = None
    gps_blink = False
    gps_lock = False
    elapsed = time.time()
    keepalive = time.time()
    skip1stbme680 = True
    while True:
        w.feed()
        try:
            gps.update()
        except MemoryError:
            print(yellow("Memory Leak !!! Reboot ..."))
            # the gps module has a nasty memory leak just ignore and reload (Gps trackings stays in tact)
            microcontroller.reset()

        if gps_lock is False:
            if gps_blink is True:
                gpsLED.value = True
                loraLED.value = False
                gps_blink = False
            else:
                time.sleep(0.1)
                gpsLED.value = False

        current = time.monotonic()
        if current - last_print >= 5:
            last_print = current
            if not gps.has_fix:
                gps_lock = False
                print(yellow("No GPS FIX!"))
                # Try again if we don't have a fix yet.
                if gps_blink is False:
                    gps_blink = True
                continue

            # We have a fix!
            w.feed()

            # epoch
            if year() == 1970:
                rtc.set_time_source(gps)
                the_rtc = rtc.RTC()

            # non synced gps year
            if year() < 2023:
                rtc.set_time_source(gps)
                the_rtc = rtc.RTC()
                print(yellow("GPS clock not synced, retrying ..."))
                continue

            if gps_lock is False:
                print(purple("We have a GPS fix && Clock is locked to GPS !)"))
                print(
                    purple(
                        "Location: LAT: "
                        + str(gps.latitude)
                        + " LON: "
                        + str(gps.longitude)
                    )
                )

            gps_lock = True
            gpsLED.value = True

            angle = -1
            speed = -1

            if gps.track_angle_deg is not None:
                angle = gps.track_angle_deg
                speed = gps.speed_knots * 1.852

            pos = aprs.makePosition(
                gps.latitude, gps.longitude, speed, angle, config.symbol
            )

            if (time.time() - keepalive) >= config.keepalive:
                keepalive = time.time()
                last_lat = None
                last_lon = None

            if (time.time() - elapsed) >= config.rate or last_lon is None:
                my_distance = distance(last_lat, last_lon, gps.latitude, gps.longitude)
                if my_distance > int(config.distance):
                    elapsed = time.time()
                    last_lat = gps.latitude
                    last_lon = gps.longitude

                    print(
                        purple(
                            "Location: LAT: "
                            + str(gps.latitude)
                            + " LON: "
                            + str(gps.longitude)
                        )
                    )

                    ts = aprs.makeTimestamp(
                        "z",
                        gps.timestamp_utc.tm_mday,
                        gps.timestamp_utc.tm_hour,
                        gps.timestamp_utc.tm_min,
                        gps.timestamp_utc.tm_sec,
                    )

                    # user comment
                    comment = config.comment

                    # telemetry
                    sequence = sequence + 1
                    if sequence > 8191:
                        sequence = 0
                    comment = (
                        comment
                        + "|"
                        + base91_encode(sequence)
                        + base91_encode(int(gps.satellites))
                    )
                    bat_voltage = int(round(get_voltage(analog_in), 2) * 100)
                    comment = comment + base91_encode(bat_voltage)
                    if shtc3 is True:
                        temperature, relative_humidity = i2c_shtc3.measurements
                        temp = int((round(temperature / 2, 2) + 25) * 100)
                        hum = int(round(relative_humidity, 0))
                        # if shtc failes ... just reload
                        if hum is None:
                            microcontroller.reset()
                        comment = comment + base91_encode(temp) + base91_encode(hum)
                        print(
                            purple(
                                "SHTC3: Temperature: "
                                + str(temperature)
                                + " Humidity: "
                                + str(relative_humidity)
                            )
                        )
                    if bme680 is True:
                        if skip1stbme680 is True:
                            print(
                                purple(
                                    "BME680: Skip first read to give BME some time to calibrate!"
                                )
                            )
                            temperature = i2c_bme680.temperature
                            relative_humidity = i2c_bme680.relative_humidity
                            skip1stbme680 = False
                        else:
                            temperature = (
                                i2c_bme680.temperature + config.bme680_tempOffset
                            )
                            relative_humidity = i2c_bme680.relative_humidity
                            temp = int((round(temperature / 2, 2) + 25) * 100)
                            hum = int(round(relative_humidity, 0))
                            comment = comment + base91_encode(temp) + base91_encode(hum)
                            print(
                                purple(
                                    "BME680: Temperature: "
                                    + str(temperature)
                                    + " Humidity: "
                                    + str(relative_humidity)
                                )
                            )
                    comment = comment + "|"

                    # altitude
                    if gps.altitude_m is not None:
                        altitude = "/A={:06d}".format(int(gps.altitude_m * 3.2808399))
                        comment = comment + altitude
                        print(purple("GPS Altitude: " + str(gps.altitude_m) + " m"))

                    # send LoRa packet
                    message = "{}>APRFGT:@{}{}{}".format(
                        config.callsign, ts, pos, comment
                    )
                    loraLED.value = True
                    if config.hasPa is True:
                        amp.value = True
                        time.sleep(0.3)
                    print(purple("LoRa send message: " + message))
                    rfm9x.send(
                        w,
                        bytes("{}".format("<"), "UTF-8")
                        + binascii.unhexlify("FF")
                        + binascii.unhexlify("01")
                        + bytes("{}".format(message), "UTF-8"),
                    )
                    if config.hasPa is True:
                        time.sleep(0.1)
                        amp.value = False
                    loraLED.value = False
                    with open("/sequence", "w") as f:
                        print(purple("Update Sequence: " + str(sequence)))
                        f.write(str(sequence))
                        f.close()
            time.sleep(0.1)
except Exception as error:
    print("Tracking err reloading: ", error)
    time.sleep(1)
    microcontroller.reset()