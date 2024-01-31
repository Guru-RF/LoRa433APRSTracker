import os
import time

import board
import digitalio
import storage
import supervisor
import usb_cdc

supervisor.runtime.autoreload = False


def file_or_dir_exists(filename):
    try:
        os.stat(filename)
        return True
    except OSError:
        return False


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

# Disable devices only if dah/dit is not pressed.
if file_or_dir_exists("/ro"):
    print("boot: ro file exists enabling drive")

    usb_cdc.enable(console=True, data=True)

    new_name = "APRSTRKR"
    storage.remount("/", readonly=False)
    m = storage.getmount("/")
    m.label = new_name
    storage.remount("/", readonly=True)
else:
    print("boot: ro file not there, disabling drive")
    storage.disable_usb_drive()
    storage.remount("/", readonly=False)

    usb_cdc.enable(console=True, data=True)
