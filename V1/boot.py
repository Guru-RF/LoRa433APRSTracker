import time

import board
import digitalio
import storage
import supervisor
import usb_cdc
from microcontroller import watchdog as w
from watchdog import WatchDogMode

supervisor.runtime.autoreload = False

# configure watchdog
w.timeout = 5
w.mode = WatchDogMode.RESET
w.feed()

supervisor.set_usb_identification("RF.Guru", "APRSTRKR")

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

# Disable AMP
amp = digitalio.DigitalInOut(board.GP2)
amp.direction = digitalio.Direction.OUTPUT
amp.value = False

# init gps
gpsRST = digitalio.DigitalInOut(board.GP12)
gpsRST.direction = digitalio.Direction.OUTPUT
gpsRST.value = False
time.sleep(1)
gpsRST.value = True
time.sleep(1)

usb_cdc.enable(console=True, data=False)

new_name = "APRSTRKR"
storage.remount("/", readonly=False)
m = storage.getmount("/")
m.label = new_name
storage.remount("/", readonly=True)
