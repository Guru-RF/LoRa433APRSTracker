# LoRA APRS Tracker Automotive 13.8v

[Assembled - RF.Guru LoRA APRS Tracker Automotive 433Mhz](https://shop.rf.guru/products/13-8v-lora-aprs-automotive-tracker-433-mhz) 400mW Power

## Supported radios

One firmware image drives both board revisions. At boot the radio is
identified over SPI and the matching driver is configured:

| Board | Radio                          | GPS                    |
|-------|--------------------------------|------------------------|
| V1    | HopeRF RFM95W / Semtech SX1276 | u-blox NEO, UBX, 9600  |
| V2    | G-NiceRF 1262MiniF27 (SX1262)  | ATGM336H, PCAS, 115200 |

V2 boards are identified by the GP15 strap, which is tied to ground. The
V2 radio adds a BUSY line on GP22 and an internal PA reaching 29 dBm.

The GPS baud rate is verified at boot rather than assumed, so a board
that disagrees with its strap still comes up; `gpsBaud` pins it if
needed. Both receivers emit `$GN` talker IDs, which TinyGPSPlus parses.

Both are configured for SF12, 125 kHz, coding rate 4/5, 8-symbol
preamble, explicit header, CRC on and sync word `0x12`, so the two
revisions are interchangeable on the air and decode on the same
gateways. The SX1262's clock source (plain crystal vs TCXO powered from
DIO3) is probed at boot, so module variants work without a rebuild.

Detection can be overridden from `config.txt` with `radioChip` and
`loraTcxo` if a board ever needs pinning.

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
- **radioChip** - `auto` (detect), or force `sx1276` / `sx1262`
- **loraTcxo** - SX1262 clock: `auto` (probe), `0` for a crystal, or the
  TCXO supply voltage such as `1.8` / `3.3`

### What `power` means depends on the board

On **V1** it is the chip's output power in dBm, capped at +20.

On **V2** the module has its own amplifier, so the chip's output is a
*drive level* into that amplifier, not the radiated power. The MiniF27
datasheet only characterises drive levels 0-9, so the firmware clamps to
that range — driving the chip to +22 dBm as a bare SX1262 would be is
13 dB past the top of the table and hard-compresses the amplifier.
Measured at the antenna pad (datasheet §8, VCC_PA = 4 V):

| Drive | Antenna port | Current |
|-------|--------------|---------|
| 0     | 9.8 dBm      | 110 mA  |
| 4     | 20.9 dBm     | 200 mA  |
| 9     | 28.7 dBm     | 460 mA  |

`paDrive` sets this and defaults to 9. `power` is a dBm figure and means
nothing to an amplifier driven by an index, so it is ignored on V2 and
the boot log says so.

The table above is the datasheet's VCC_PA = 4 V column, which matches
this board: VCCFILTER is 4.85 V behind a 1N4007F, leaving roughly 4.05 V
at full drive. The figure in the boot log is still an estimate — the
drive level is what is actually programmed. On V2 the module output
reaches the SMA through a harmonic low-pass filter, so there is no
second amplifier in the chain.

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

## Hardware diagnostics

`tools/chipprobe/` holds two standalone firmwares for bringing up a new
board. Neither transmits or enables the external PA.

```console
cd tools/chipprobe

# Survey an unknown board: which radio is fitted, and which GPIO carries
# BUSY / DIO1 / DIO3 (found by watching every free pin while the radio
# is made to assert them).
pio run -e probe

# Run the tracker's real radio boot path and read the settings back out
# of the chip, without keying up.
pio run -e radiotest
```

Flash either with `picotool load -x .pio/build/<env>/firmware.uf2`.

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
