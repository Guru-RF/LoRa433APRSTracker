import time
import board
import busio
import adafruit_gps
import digitalio
import adafruit_rfm9x
import binascii
from APRS import APRS
import supervisor
from microcontroller import watchdog as w
from watchdog import WatchDogMode
import config

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
if config.type.lower() is 'highpower':
    rfm9x.tx_power = 23

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
        lat = round(gps.latitude, 2)
        lon = round(gps.longitude, 2)

        if ((time.time()-elapsed) >= config.rate or last_lon is None):
            elapsed = time.time()
            if ((time.time()-keepalive) >= config.keepalive):
                keepalive = time.time()
                last_lat = 0
                last_lon = 0
            if (last_lat is not lat or last_lon is not lon):
                last_lon = lon
                last_lat = lat

                ts = aprs.makeTimestamp('z',gps.timestamp_utc.tm_mday,gps.timestamp_utc.tm_hour,gps.timestamp_utc.tm_min,gps.timestamp_utc.tm_sec)

                comment = config.comment
                if gps.altitude_m is not None:
                    altitude = "/A={:06d}".format(int(gps.altitude_m*3.2808399))
                    comment = comment + altitude

                message = "{}>APRFGT:@{}{}{}".format(config.callsign, ts, pos, comment)
                loraLED.value = True
                if config.type.lower() is 'highpower':
                    amp.value = True
                rfm9x.send(
                    bytes("{}".format("<"), "UTF-8") + binascii.unhexlify("FF") + binascii.unhexlify("01") +
                    bytes("{}".format(message), "UTF-8")
                )
                if config.type.lower() is 'highpower':
                    amp.value = False
                loraLED.value = False
                gps = adafruit_gps.GPS(uart, debug=False) 
        w.feed()
        time.sleep(0.1)
