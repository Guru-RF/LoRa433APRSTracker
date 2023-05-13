import time
import board
import busio
import adafruit_gps
import digitalio
import adafruit_rfm9x
from APRS import APRS

# APRS encoder
aprs = APRS()

# LoRa APRS frequency
RADIO_FREQ_MHZ = 433.775
CS = digitalio.DigitalInOut(board.GP21)
RESET = digitalio.DigitalInOut(board.GP20)
spi = busio.SPI(board.GP18, MOSI=board.GP19, MISO=board.GP16)

rfm9x = adafruit_rfm9x.RFM9x(spi, CS, RESET, RADIO_FREQ_MHZ, baudrate=1000000)
rfm9x.tx_power = 5
rfm9x.signal_bandwidth = 125000
rfm9x.coding_rate = 5
rfm9x.spreading_factor = 7

#rfm9x.send(bytes("message number {}".format(counter), "UTF-8"))

uart = busio.UART(board.GP4, board.GP5, baudrate=9600, timeout=10)

gps = adafruit_gps.GPS(uart, debug=False)  # Use UART/pyserial
gps.send_command(b"PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0")
gps.send_command(b"PMTK220,1000")

last_print = time.monotonic()
last_lat = None
last_lon = None
while True:
    gps.update()
    # Every second print out current location details if there's a fix.
    current = time.monotonic()
    if current - last_print >= 1.0:
        last_print = current
        if not gps.has_fix:
            # Try again if we don't have a fix yet.
            print("Waiting for fix...")
            continue
        # We have a fix! (gps.has_fix is true)
        # Print out details about the fix like location, date, etc.a
        if last_lat is not gps.latitude and last_lon is not gps.longitude:
            print("Moved!")
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
            type = '/j'
            comment = "LoRa APRS RF.Guru"
            ts = aprs.makeTimestamp('h',gps.timestamp_utc.tm_hour,gps.timestamp_utc.tm_min,gps.timestamp_utc.tm_sec)
            pos = aprs.makePosition(gps.latitude,gps.longitude,-1,-1,-1,type)

            #DJ0ABR-7>APLT00,WIDE1-1:!4849.27N/01307.72E[/A=001421LoRa Tracker
            message = "{}>APLORA,WIDE1-1:@{}{}{}".format(callsign, ts, pos, comment)
            print(message)
            rfm9x.send(bytes("{}".format(message), "UTF-8"))
            time.sleep(10)
            print("done sending!")