# LoRA APRS Tracker Automotive 13.8v

[Assembled - RF.Guru LoRA APRS Tracker Automotive 433Mhz](https://shop.rf.guru/products/13-8v-lora-aprs-automotive-tracker-433-mhz) 400mW Power

# Installation

## Quick Install (pre-built firmware)

1. Download `LoRa433APRSTracker.uf2` from the [latest release](https://github.com/Guru-RF/LoRa433APRSTracker/releases/latest)
2. Connect the device via USB-C (**do not** connect the 13.8v Powerpole)
3. Enter UF2 bootloader mode:
   - Hold the reset button on the PCB while plugging in USB-C, **or**
   - Run `./reset.sh /dev/tty.usbmodem*` (installs pyserial automatically)
4. A drive named `RPI-RP2` will appear - copy the `.uf2` file to it
5. The device reboots automatically and a new drive named `APRSTRKR` appears
6. Edit `config.txt` on the drive (see Configuration below)
7. Eject the drive - the device reboots with your settings

## Configuration

After flashing, the device presents as a USB drive with a `config.txt` file. Edit it to configure:

- **callsign** - Your APRS callsign (e.g. `ON9RFG-1`)
- **profile** - SmartBeacon profile: `car`, `bike`, or `hiker`
- **power** - TX power 5-23 dBm
- **triggerVoltageCall** - Callsign to alert on low voltage
- **loraFrequency** - LoRa frequency in MHz (default 433.775)
- **i2cEnabled** - Enable I2C sensors (`true`/`false`)
- **i2cDevice** - Sensor type: `BME680` or `SHTC3`

Eject the USB drive to apply changes (device reboots automatically).

## Firmware Update

To update to a new firmware version, either:

- Repeat the Quick Install steps above with the new `.uf2` file, **or**
- Edit `config.txt` on the USB drive, replace all content with just `firmwareupdate`, and eject - the device will reboot into UF2 bootloader mode ready for the new firmware

## Factory Reset

Copy `flash_nuke.uf2` to the `RPI-RP2` drive to erase all flash, then reflash the firmware.

# Building from Source

Requires [PlatformIO](https://platformio.org/).

```console
pio run
```

The firmware will be at `.pio/build/pico/firmware.uf2`.

# Serial Debugging/Console

Install tio: [https://github.com/tio/tio](https://github.com/tio/tio)

```console
tio --auto-connect new
```

The console shows colored output with GPS status, LoRa TX frames, and voltage monitoring.

<img width="938" alt="TrackerTOP" src="https://github.com/Guru-RF/LoraAPRStracker/assets/1251767/c3a32cc5-92fe-420b-a335-53400f411a51">
<img width="1076" alt="TrackerBottom" src="https://github.com/Guru-RF/LoraAPRStracker/assets/1251767/2ef5376d-9d41-4aac-892e-fea3d2fedd85">

# Warnings

The PA only runs on 13.8V via the PP45 connector. If you power the unit from USB, the power amplifier is not powered/active, so there will be no amplified output.
