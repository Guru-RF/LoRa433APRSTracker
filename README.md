# LoRA APRS Tracker Automotive 13.8v

[Assembled - RF.Guru LoRA APRS Tracker Automotive 433Mhz](https://shop.rf.guru/products/2023-pa-521) 512mW Power

[Kit - RF.Guru LoRA APRS Tracker Automotive 433Mhz](https://shop.rf.guru/products/2023-k-521) 512mW Power

[PCB - RF.Guru LoRA APRS Tracker Automotive 433Mhz](https://shop.rf.guru/products/2023-p-521) 512mW Power

# Installation procedure #
Connect the device to our computer using a UBC-C cable; a drive labeled 'RPI-RP2' will appear. 

[Download the code](https://github.com/Guru-RF/LoRa433APRSTracker/archive/refs/heads/main.zip)

Copy the 'adafruit-circuitpython-raspberry_pi_pico-en_US-9.?.uf2' file to the drive and patiently wait for it to reboot! 

Once it restarts, you should see a new drive named 'CIRCUITPY' 

Transfer the library folder (src/lib), followed by the src/boot.py file. Adjust the settings in the src/config.py (modify call/settings) and transfer it to the 'CIRCUITPY' drive. 

Finally, transfer the src/code.py file!

<img width="938" alt="TrackerTOP" src="https://github.com/Guru-RF/LoraAPRStracker/assets/1251767/c3a32cc5-92fe-420b-a335-53400f411a51">
<img width="1076" alt="TrackerBottom" src="https://github.com/Guru-RF/LoraAPRStracker/assets/1251767/2ef5376d-9d41-4aac-892e-fea3d2fedd85">

# Serial Debugging/Console (TIO)

First install tio [https://github.com/tio/tio](https://github.com/tio/tio)

```console
tio --auto-connect new
```

```console
tio --auto-connect latest
```

```console
tio /dev/tty....
```

![console cast](https://github.com/Guru-RF/LoRa433APRSTracker/assets/1251767/9bcc6d71-46d9-40a1-86c3-337bbb93e6df)

# Resetting to factory defaults

Via the console ... press control-c and past this

```console
import microcontroller
microcontroller.on_next_reset(microcontroller.RunMode.UF2)
microcontroller.reset()
```

Follow the installation procedure on top of this page !

# Reset by hand

As a last restort !
Open the device ... press the tiny reset button on the pcb and connect usb-c cable whilst pressing the reset button.

Follow the installation procedure on top of this page !

# Remarks

lib/adafruit_rfm9x.mpy is heavily modified to work with loraAPRS
you can find the modifications here:
https://github.com/Guru-RF/RF_Guru_RFM9x

lib/APRS.mpy:
you can find the uber minimalistic APRS lib over here
https://github.com/Guru-RF/circuitpython-APRS

