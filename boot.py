import usb_cdc
import board
import storage
import time
import digitalio

# init gps
gpsRST = digitalio.DigitalInOut(board.GP12)
gpsRST.direction = digitalio.Direction.OUTPUT
gpsRST.value = False
time.sleep(0.5)
gpsRST.value = True
time.sleep(0.5)

btn = digitalio.DigitalInOut(board.GP6)
btn.direction = digitalio.Direction.INPUT
btn.pull = digitalio.Pull.UP

btn2 = digitalio.DigitalInOut(board.GP15)
btn2.direction = digitalio.Direction.INPUT
btn2.pull = digitalio.Pull.UP

# Disable devices only if dah/dit is not pressed.
if btn.value is True and btn2.value is True:
    print(f"boot: button not pressed, disabling drive")
    storage.disable_usb_drive()
    storage.remount("/", readonly=False)

    usb_cdc.enable(console=True, data=False)
else:
    print(f"boot: button pressed, enable console, enabling drive")

    usb_cdc.enable(console=True, data=False)

    new_name = "APRSTRKR"
    storage.remount("/", readonly=False)
    m = storage.getmount("/")
    m.label = new_name
    storage.remount("/", readonly=True)