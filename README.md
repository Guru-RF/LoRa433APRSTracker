# LoRA APRS Tracker Automotive 13.8v

[Assembled - RF.Guru LoRA APRS Tracker Automotive 433Mhz](https://shop.rf.guru/products/2023-pa-521) 512mW Power

[Kit - RF.Guru LoRA APRS Tracker Automotive 433Mhz](https://shop.rf.guru/products/2023-k-521) 512mW Power

[PCB - RF.Guru LoRA APRS Tracker Automotive 433Mhz](https://shop.rf.guru/products/2023-p-521) 512mW Power

# Firmware

Built with PlatformIO (Arduino C++ for RP2040).

The stable CircuitPython version is preserved in `V1/`.

# Installation

## Building

```console
pio run
```

## Flashing

Connect the device via USB-C (do not connect 13.8v Powerpole).

To enter UF2 bootloader mode, use `reset.py` or hold the reset button while connecting USB-C:

```console
python3 reset.py /dev/tty.usbmodem*
```

Then copy the firmware:

```console
cp .pio/build/pico/firmware.uf2 /Volumes/RPI-RP2/
```

## Uploading Filesystem (first time)

Upload the default config to the device filesystem:

```console
pio run --target uploadfs
```

## Configuration

After flashing, the device presents as a USB drive. Edit `config.txt` on the drive to configure:

- **callsign** - Your APRS callsign
- **profile** - SmartBeacon profile: `car`, `bike`, or `hiker`
- **power** - TX power 5-23 dBm
- **triggerVoltageCall** - Callsign to alert on low voltage
- And more (see comments in config.txt)

Reboot after editing.

## Factory Reset

Copy `flash_nuke.uf2` to the RPI-RP2 drive to erase all flash, then reflash firmware.

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

![console cast](https://github.com/Guru-RF/LoRa433APRSTracker/assets/1251767/6fd05385-3f13-4b30-8b80-1ce499a2039c)

# Warnings

Just a quick note: the PA only runs on 13.8 V via the PP45 connector. If you power the unit from USB, the power amplifier is not powered/active, so there will be no amplified output.
