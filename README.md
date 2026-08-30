# LoRA APRS Tracker Automotive 13.8v

[Assembled - RF.Guru LoRA APRS Tracker Automotive 433Mhz](https://shop.rf.guru/products/13-8v-lora-aprs-automotive-tracker-433-mhz) 400mW Power

## Supported radios

One firmware image drives both board revisions. At boot the radio is
identified over SPI and the matching driver is configured:

| Board | Radio                          | GPS                    | Output  |
|-------|--------------------------------|------------------------|---------|
| V1    | HopeRF RFM95W / Semtech SX1276 | u-blox NEO, UBX, 9600  | ~400 mW |
| V2    | G-NiceRF 1262MiniF27 (SX1262)  | ATGM336H, PCAS, 115200 | ~500 mW |

Both revisions are CE-RED certified.

The revision is identified from the radio itself, read over SPI. The
GP15 strap is not used for it - measurement showed it reads low on V1
hardware as well, so it identifies nothing. The V2 radio adds a BUSY
line on GP22 and an internal PA reaching 29 dBm.

The GPS is brought up without needing to know the revision either: the
baud rate is found by listening, and both the u-blox (UBX) and ATGM336H
(PCAS) command sets are sent, since each is ignored by the other module.
`gpsBaud` pins the rate if a board ever needs it. Both receivers emit
`$GN` talker IDs, which TinyGPSPlus parses.

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
- **power** - TX power in dBm (bare modules; see below for V2)
- **triggerVoltageCall** - Callsign to alert on low voltage
- **loraFrequency** - LoRa frequency in MHz (default 433.775)
- **i2cEnabled** - Enable I2C sensors (`true`/`false`)
- **i2cDevice** - Sensor type: `BME680` or `SHTC3`
- **radioChip** - `auto` (detect), or force `sx1276` / `sx1262`
- **loraTcxo** - SX1262 clock: `auto` (probe), `0` for a crystal, or the
  TCXO supply voltage such as `1.8` / `3.3`
- **paDrive** - chip dBm into a module PA, -9..22 (V2 only)
- **gpsBaud** - `0` to detect, or pin it (`9600` / `115200`)
- **usbPaDrive** - drive used while a computer has the tracker enumerated,
  -9..14 (default `-9`). Higher will not ramp on USB power, so the firmware
  clamps it
- **usbTxInhibit** - refuse to transmit at all while a computer is attached,
  instead of dropping to `usbPaDrive` (`true`/`false`, default `false`)

### Transmitting while connected to a computer

A V2 board cannot transmit at rated drive while a computer has it enumerated.
The amplifier's ramp does not complete, the radio driver waits on BUSY with no
timeout, and the tracker stops dead mid-frame. Measured on the bench: it hangs
at `paDrive` 20 and 22, works at 14 and below, and works at 22 the moment
USB-C is unplugged and it runs from the Powerpole alone.

So the drive is chosen per frame. Rated `paDrive` when nothing is enumerated;
`usbPaDrive` when something is. That follows the cable with no reboot - unplug
USB and the very next beacon goes out at full power.

Only an actual USB host counts. A charger and the 13.8 V Powerpole never
enumerate anything, so a deployed tracker always runs at rated drive.

The default `usbPaDrive=-9` is the SX1262's floor into the module amplifier,
and it is still far more than enough on a bench: an iGate 8 m away decodes it
at around -56 dBm with 6 dB of SNR. The point is that the tracker stays
audible while you work on it, so you can watch packets arrive at your iGate
with the console attached and the config drive mounted.

`usbTxInhibit=true` restores the older behaviour - silence rather than low
power - for a port that cannot supply even the floor drive. The tracker then
says so on the console and winks the power LED once every 5 seconds. The LoRa
LED is left alone either way, so a blinking LoRa LED always means the
transmitter is keyed.

### What `power` means depends on the board

On **V1** it is the chip's output power in dBm, capped at +20.

On **V2** the module has its own amplifier, so the chip's output is the
*drive* into that amplifier rather than the radiated power. That has its
own key, `paDrive`, in dBm from -9 to 22, and `power` is ignored.

`paDrive` defaults to 22 — the die's maximum — because that is what the
module needs for its rated output. The datasheet's "Chip Output Level
0..9" column is an index across the die's range, not a figure in dBm;
reading it as dBm implies the module's gain grows by 10 dB with drive,
which no amplifier does. Bench measurement confirmed the index reading:
full die drive gave the highest output, and the current draw then
matched the datasheet's efficiency.

What actually reaches the antenna depends on VCC_PA (26.6 dBm at 3.3 V
rising to 29.8 dBm at 5.0 V) and on the filtering after the module, so
the firmware reports what it programmed rather than predicting power it
cannot measure.

Two V2 part choices matter to output power, and both are load-bearing
rather than incidental. The series diode feeding VCC_PA must be a
Schottky — a PN part costs ~0.5 V more at these currents, about 1 dB.
The antenna-port ESD device must stay out of the way of the RF swing:
800 mW into 50 ohm is 8.9 V peak, so a 5 V TVS clamps the transmitter
and caps output regardless of drive.

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

`tools/chipprobe/` holds several standalone firmwares for bringing up a
new board. Only the `-tx` and `cwtest` builds transmit; the rest are
passive and never enable the PA. They are separate binaries and ignore
`config.txt`, so they always run at their own compiled-in drive rather than
the tracker's USB-aware one.

```console
cd tools/chipprobe

# Survey an unknown board: which radio is fitted, and which GPIO carries
# BUSY / DIO1 / DIO3 (found by watching every free pin while the radio
# is made to assert them).
pio run -e probe

# Run the tracker's real radio boot path and read the settings back out
# of the chip, without keying up.
pio run -e radiotest

# Survey the GPS: sweep baud rates, dump raw NMEA, tally talker IDs.
pio run -e gpsdump

# Hold a carrier at a series of PA settings so a power meter has
# something steady to read. Antenna or dummy load required.
pio run -e cwtest
```

Flash either with `picotool load -x .pio/build/<env>/firmware.uf2`.

# Serial Debugging/Console

Install tio: [https://github.com/tio/tio](https://github.com/tio/tio)

```console
tio --auto-connect new
```

The console shows colored output with GPS status, LoRa TX frames, and voltage monitoring.
With the console attached the tracker transmits at reduced drive - see
[Transmitting while connected to a computer](#transmitting-while-connected-to-a-computer).

<img width="938" alt="TrackerTOP" src="https://github.com/Guru-RF/LoraAPRStracker/assets/1251767/c3a32cc5-92fe-420b-a335-53400f411a51">
<img width="1076" alt="TrackerBottom" src="https://github.com/Guru-RF/LoraAPRStracker/assets/1251767/2ef5376d-9d41-4aac-892e-fea3d2fedd85">

# Warnings

**V1 boards:** the PA only runs on 13.8V via the PP45 connector. If you
power the unit from USB, the power amplifier is not powered/active, so
there will be no amplified output.

**V2 boards:** the amplifier is inside the radio module and is fed from
the on-board buck regulator, so full output is available on USB-C as
well as on the Powerpole. USB-C and the Powerpole may be connected at
the same time - the board protects against it. Rated output needs the
Powerpole or a plain charger, though: with a computer enumerated the
firmware drops to `usbPaDrive`.

**Car head units are USB hosts.** A head unit or media player will
enumerate the tracker and hold it at `usbPaDrive` for the whole journey.
Power an in-car tracker from the Powerpole or a plain charger rather than
a head unit's data port, or raise `usbPaDrive` if you must use one.

Never transmit without an antenna or a 50 ohm dummy load on the SMA.
