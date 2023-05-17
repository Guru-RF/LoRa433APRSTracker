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

# Reset GPS
gpsRST = digitalio.DigitalInOut(board.GP12)
gpsRST.direction = digitalio.Direction.OUTPUT
gpsRST.value = False
print("reseting gps!")
time.sleep(0.3)
w.feed()
gpsRST.value = True
w.feed()

print("gps init!")
#w.feed()

# APRS encoder
aprs = APRS()

# LoRa APRS frequency
RADIO_FREQ_MHZ = 433.775
CS = digitalio.DigitalInOut(board.GP21)
RESET = digitalio.DigitalInOut(board.GP20)
spi = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)

rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ, baudrate=1000000)
rfm9x.tx_power = 23 # 5 min 23 max

uart = busio.UART(board.GP4, board.GP5, baudrate=9600, timeout=10)

gps = adafruit_gps.GPS(uart, debug=True)  # Use UART/pyserial

last_print = time.monotonic()
last_lat = None
last_lon = None
gps_blink = False
gps_lock = False
lora_blink = False
while True:
    w.feed()
    try:
        gps.update()
    except MemoryError:
        supervisor.reload()
    time.sleep(1)

    if gps_lock is True:
        if lora_blink is True:
            loraLED.value = True
            lora_blink = False
        else:
            #time.sleep(0.3)
            loraLED.value = False
    if gps_lock is False:
        if gps_blink is True:
            gpsLED.value = True
            gps_blink = False
        else:
            #time.sleep(0.3)
            gpsLED.value = False
    # Every second print out current location details if there's a fix.
    current = time.monotonic()
    if current - last_print >= 5:
        last_print = current
        if not gps.has_fix:
            gps_lock = False
            # Try again if we don't have a fix yet.
            #print("Waiting for fix...")
            if gps_blink is False:
                gps_blink = True
            w.feed()
            continue
        gps_lock = True
        gpsLED.value = True
        if lora_blink is False:
            lora_blink = True
        w.feed()
        # We have a fix! (gps.has_fix is true)
        # Print out details about the fix like location, date, etc.a
        #w.feed()
        if last_lat is not gps.latitude and last_lon is not gps.longitude:
            #print("Moved!")
            last_lat = gps.latitude
            last_lon = gps.longitude
            print("=" * 40)  # Print a separator line.
            print(
                "Fix timestamp: {}/{}/{} {:02}:{:02}:{:02}".format(
                    gps.timestamp_utc.tm_mon,  # Grab parts of the time from the
                    gps.timestamp_utc.tm_mday,  # struct_time object that holds
                    gps.timestamp_utc.tm_year,  # the fix time.  Note you might
                    gps.timestamp_utc.tm_hour,  # not get all data like year, day,
                    gps.timestamp_utc.tm_min,  # month!
                    gps.timestamp_utc.tm_sec,
                )
            )
            print("Latitude: {0:.6f} degrees".format(gps.latitude))
            print("Longitude: {0:.6f} degrees".format(gps.longitude))
            print("Fix quality: {}".format(gps.fix_quality))
            # Some attributes beyond latitude, longitude and timestamp are optional
            # and might not be present.  Check if they're None before trying to use!
            if gps.satellites is not None:
                print("# satellites: {}".format(gps.satellites))
            if gps.altitude_m is not None:
                print("Altitude: {} meters".format(gps.altitude_m))
            if gps.speed_knots is not None:
                print("Speed: {} knots".format(gps.speed_knots))
            if gps.track_angle_deg is not None:
                print("Track angle: {} degrees".format(gps.track_angle_deg))
            if gps.horizontal_dilution is not None:
                print("Horizontal dilution: {}".format(gps.horizontal_dilution))
            if gps.height_geoid is not None:
                print("Height geo ID: {} meters".format(gps.height_geoid))

            callsign = "ON3URE-11"
            type = '/k'
            comment = "RF.Guru"
            ts = aprs.makeTimestamp('h',gps.timestamp_utc.tm_hour,gps.timestamp_utc.tm_min,gps.timestamp_utc.tm_sec)
            pos = aprs.makePosition(gps.latitude,gps.longitude,-1,-1,-1,type)

            message = "{}>APLT00,WIDE1-1:@{}{}{}".format(callsign, ts, pos, comment)
            print(message)
            loraLED.value = True
	    print(rfm9x.send(
                bytes("{}".format("<"), "UTF-8") + binascii.unhexlify("FF") + binascii.unhexlify("01") +
                bytes("{}".format(message), "UTF-8")
	    ))
            w.feed()
	    time.sleep(3)
            w.feed()
            loraLED.value = False
            print("done sending!")
        #else:
            #print(str(gps.timestamp_utc.tm_sec) + " not moved!")
