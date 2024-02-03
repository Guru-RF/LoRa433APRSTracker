# LoRA APRS Tracker Automotive 13.8v

[RF.Guru LoRA APRS Tracker Automotive 433Mhz](https://rf.guru/2023-k-521) 1/2W Power

# Installation procedure #
Connect the device to our computer using a UBC-C cable; a drive labeled 'RPI-RP2' will appear. 

Copy the 'adafruit-circuitpython-raspberry_pi_pico-en_US-8.2.9.uf2' file to the drive and patiently wait for it to reboot! 

Once it restarts, you should see a new drive named 'CIRCUITPY' 

Transfer the library folder (src/lib), followed by the src/boot.py file. Adjust the settings in the src/config.py (modify call/settings) and transfer it to the 'CIRCUITPY' drive. 

Finally, transfer the src/code.py file!

<img width="938" alt="TrackerTOP" src="https://github.com/Guru-RF/LoraAPRStracker/assets/1251767/c3a32cc5-92fe-420b-a335-53400f411a51">
<img width="1076" alt="TrackerBottom" src="https://github.com/Guru-RF/LoraAPRStracker/assets/1251767/2ef5376d-9d41-4aac-892e-fea3d2fedd85">

# Remarks

lib/adafruit_rfm9x.mpy is heavily modified to work with loraAPRS
you can find the modifications here:
https://github.com/Guru-RF/RF_Guru_RFM9x

lib/APRS.mpy:
you can find the uber minimalistic APRS lib over here
https://github.com/Guru-RF/circuitpython-APRS

