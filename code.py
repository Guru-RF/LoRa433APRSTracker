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

# APRS encoder
aprs = APRS()

# LoRa APRS frequency
RADIO_FREQ_MHZ = 433.775
CS = digitalio.DigitalInOut(board.GP21)
RESET = digitalio.DigitalInOut(board.GP20)
spi = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)

# Lora Module
rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ, baudrate=1000000)
rfm9x.tx_power = 23 # 5 min 23 max

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

# Disable al other GPS data
Disable_NMEA = bytes ([
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x24, # GxGGA
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x2B, # GxGLL
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x02, 0x32, # GxGSA
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x03, 0x39, # GxGSV
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47, # GxVTG
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x05, 0x47, # GxVTG
        ])
gps.send_command(Disable_NMEA)
time.sleep(0.1)

# Disable UBX data
Disable_UBX = bytes ([
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x12, 0xB9, #NAV-POSLLH
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0xC0, #NAV-STATUS
        0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0x01, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x13, 0xC0, #NAV-STATUS
])
gps.send_command(Disable_UBX)
time.sleep(0.1)

# Start Tracking
last_print = time.monotonic()
last_pos = None
gps_blink = False
gps_lock = False
elapsed = 0
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
        elapsed=elapsed+1
            
        angle = -1
        if gps.track_angle_deg is not None:
            angle = gps.track_angle_deg
            
        pos = aprs.makePosition(gps.latitude,gps.longitude,(gps.speed_knots*1.852),angle,gps.altitude_m,config.symbol)

        if (last_pos is not pos and elapsed is config.rate) or last_pos is None:
            last_pos = pos
            elapsed = 0

            ts = aprs.makeTimestamp('h',gps.timestamp_utc.tm_hour,gps.timestamp_utc.tm_min,gps.timestamp_utc.tm_sec)

            message = "{}>APRS:@{}{}{}".format(config.callsign, ts, pos, config.comment)
            loraLED.value = True
            rfm9x.send(
                bytes("{}".format("<"), "UTF-8") + binascii.unhexlify("FF") + binascii.unhexlify("01") +
                bytes("{}".format(message), "UTF-8")
            )
            loraLED.value = False
            gps = adafruit_gps.GPS(uart, debug=False) 
        w.feed()
        time.sleep(0.1)
