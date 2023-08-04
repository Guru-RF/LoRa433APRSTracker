import time
import board
import busio
import adafruit_gps
import digitalio
import adafruit_rfm9x
from analogio import AnalogIn
import binascii
from APRS import APRS
import supervisor
from microcontroller import watchdog as w
from watchdog import WatchDogMode
from math import sin, cos, sqrt, atan2, radians, log, ceil
import config

def distance(lat1, lon1, lat2, lon2):
    if lat1 is None:
        return 999999
    radius = 6371  # km

    dlat = radians(lat2 - lat1)
    dlon = radians(lon2 - lon1)
    a = (sin(dlat / 2) * sin(dlat / 2) +
         cos(radians(lat1)) * cos(radians(lat2)) *
         sin(dlon / 2) * sin(dlon / 2))
    c = 2 * atan2(sqrt(a), sqrt(1 - a))
    return int(round(radius * c * 1000,0)) 


def get_voltage(pin):
    if config.pa is True:
        return ((pin.value * 3.3) / 65536) + 10.6 + 0.7
    else:
        return ((pin.value * 3.3) / 65536) * 2


def base91_encode(number):
    text = []

    if number < 0:
        raise ValueError("Expected number to be positive integer")
    elif number > 0:
        max_n = ceil(log(number) / log(91))

        for n in range(int(max_n), -1, -1):
            quotient, number = divmod(number, 91**n)
            text.append(chr(33 + quotient))

    text = "".join(text).lstrip('!')
    if len(text) == 1:
        text = "!" + text
    
    return text    


# Voltage adc
analog_in = AnalogIn(board.GP27)
if config.pa is False:
    analog_in = AnalogIn(board.GP26)

# Configure Watchdog
w.mode = WatchDogMode.RESET
w.timeout=5 # Set a timeout of 5 seconds
w.feed()

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

# APRS encoder
aprs = APRS()

# LoRa APRS frequency
RADIO_FREQ_MHZ = 433.775
CS = digitalio.DigitalInOut(board.GP21)
RESET = digitalio.DigitalInOut(board.GP20)
spi = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)

# Lora Module
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ, baudrate=1000000)
rfm9x.tx_power = config.power # 5 min 23 max
if config.pa is True:
    rfm9x.tx_power = 19

# GPS Module (uart)
uart = busio.UART(board.GP4, board.GP5, baudrate=9600, timeout=10, receiver_buffer_size=1024)
gps = adafruit_gps.GPS(uart, debug=False) 

# Set GPS speed to 1HZ
Speed = bytes ([
        0xB5,0x62,0x06,0x08,0x06,0x00,0xE8,0x03,0x01,0x00,0x01,0x00,0x01,0x39 #1Hz
        #0xB5, 0x62, 0x06, 0x08, 0x06, 0x00, 0xC8, 0x00, 0x01, 0x00, 0x01, 0x00, 0xDE, 0x6A 5hz
])
gps.send_command(Speed)
time.sleep(0.1)

## Disable UBX data
Disable_UBX = bytes ([
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xB9, #NAV-POSLLH
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0xC0, #NAV-STATUS
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0xC0, #NAV-STATUS
])
gps.send_command(Disable_UBX)
time.sleep(0.1)

# Start Tracking
last_print = time.monotonic()
last_lat = None
last_lon = None
gps_blink = False
gps_lock = False
elapsed = time.time()
keepalive = time.time()
sequence = 0
params = False
while True:
    w.feed()
    try:
        gps.update()
    except MemoryError:
        # the gps module has a nasty memory leak just ignore and reload (Gps trackings stays in tact)
        supervisor.reload()

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
            # Try again if we don't have a fix yet.
            if gps_blink is False:
                gps_blink = True
            w.feed()
            continue
        gps_lock = True
        gpsLED.value = True

        # We have a fix!
        angle = -1
        speed = -1
        
        if gps.track_angle_deg is not None:
            angle = gps.track_angle_deg
            speed = gps.speed_knots*1.852 
            
        pos = aprs.makePosition(gps.latitude,gps.longitude,speed,angle,config.symbol)

        if ((time.time()-keepalive) >= config.keepalive):
            keepalive = time.time()
            last_lat = None
            last_lon = None

        if ((time.time()-elapsed) >= config.rate or last_lon is None):
            my_distance = distance(last_lat,last_lon,gps.latitude,gps.longitude)
            if my_distance > int(config.distance):
                elapsed = time.time()
                last_lat = gps.latitude
                last_lon = gps.longitude

                ts = aprs.makeTimestamp('z',gps.timestamp_utc.tm_mday,gps.timestamp_utc.tm_hour,gps.timestamp_utc.tm_min,gps.timestamp_utc.tm_sec)

                comment = config.comment

                bat_voltage = 0
                if config.voltage is True:
                    bat_voltage = int(round(get_voltage(analog_in),2)*100)
                # stats go here we need to safe sequence to disk ... 
                sequence=sequence+1
                #comment = comment + "|" + base91_encode(sequence) + base91_encode(int(gps.satellites)) + base91_encode(bat_voltage) + "|"
                if gps.altitude_m is not None:
                    altitude = "/A={:06d}".format(int(gps.altitude_m*3.2808399))
                    comment = comment + altitude

                #send this each boot ?
                # if params is False:
                # params = True 
                #data = "PARM.Satelites,Battery,Temperature"
                #data = "UNIT.Nr,Vdc,C"
                #data = "EQNS.0,1,0,0,0.01,0,0,1,0"
                #message = "{}>APRFGT::{}:{}".format(config.callsign, config.callsign, data)
                
                message = "{}>APRFGT:@{}{}{}".format(config.callsign, ts, pos, comment)
                loraLED.value = True
                if config.pa is True:
                    amp.value = True
                    time.sleep(0.3)
                rfm9x.send(
                    bytes("{}".format("<"), "UTF-8") + binascii.unhexlify("FF") + binascii.unhexlify("01") +
                    bytes("{}".format(message), "UTF-8")
                )
                if config.pa is True:
                    time.sleep(0.1)
                    amp.value = False
                loraLED.value = False
                gps = adafruit_gps.GPS(uart, debug=False) 
        w.feed()
        time.sleep(0.1)
