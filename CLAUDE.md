# LoRa433APRSTracker

RP2040 APRS tracker firmware, Arduino/C++ on the earlephilhower core. One
binary drives both board revisions. Start with README.md for what the product
is and how it is configured; this file holds what the README does not - the
non-obvious constraints, and the open bugs.

## Build and verify

```console
pio run -e pico                 # build; the only environment
pio run -e pico -t upload       # flash over USB (picotool, 1200 bps touch)
```

`pio` is often not on PATH when PlatformIO was installed via its own
installer - `~/.platformio/penv/bin/pio` is the fallback. A clean build is
zero warnings; treat any warning as introduced by your change. Do not delete
`.pio/`: on a machine with no global library store it is the only copy of the
dependencies, and removing it turns an offline build into a networked one.

`pio run -t uploadfs` overwrites `config.txt` on the device and will destroy
the owner's callsign and settings. A firmware `.uf2` writes only the sketch
region, so ordinary flashing leaves the filesystem alone. Back up
`/Volumes/APRSTRKR/config.txt` before anything that touches the filesystem.

## Things that look like bugs but are not

- **GP15 is not a board-revision strap.** It measures low on V1 hardware too,
  so it identifies nothing. The revision comes from the radio, read over SPI
  in `src/radio.cpp`. `include/pins.h` keeps it documented and commented out
  so nobody wires it into a decision again.
- **The radio interrupt line is not routed** on either revision, so transmit
  is polled rather than interrupt-driven. The poll loop feeds the watchdog,
  which the old blocking `LoRa.endPacket()` could not.
- **RadioLib waits on BUSY with no timeout** after SetTx. A module whose PA
  supply is absent spins there; the 5 s watchdog is the only backstop.
- **`config.txt` keys that are absent fall back to firmware defaults.** Older
  files on deployed devices lack the newer keys, so a default change silently
  changes behaviour in the field.
- **`CONFIG_TEMPLATE` in `include/config_file.h` is dead code.** The live
  default file is written line by line in `configCreateDefault()`. Keep them
  in step or delete the template.

## Transmit inhibit while a USB host is attached

`src/main.cpp` refuses to key the radio while a computer has the tracker
enumerated, because some USB ports cannot hold 5 V through a multi-second SF12
frame. Detection is `tud_mounted()` (USB SET_CONFIGURATION), not the
mass-storage mount - there is no dependable mount signal, as FatFSUSB's
`onPlug` only fires on a SCSI START STOP UNIT load that no OS sends.

Ejecting the drive latches `"EJCT"` into `watchdog_hw->scratch[2]` and reboots;
the next boot reports no medium so nothing can re-mount it, and transmission is
enabled for the rest of that power session. Scratch survives a soft reset and
is cleared by power-on, which is what makes that work. `scratch[2]` is the only
free word - `[0..1]` carry `reset_usb_boot()` arguments and `[4..7]` are the
SDK's reboot vector and watchdog magic.

The gate in `loop()` must stay **above** `sb.shouldBeacon()`. Suppressing
between that call and `sb.updateAfterBeacon()` leaves `_lastLat` at 999, which
makes `shouldBeacon()` true on every 50 ms pass and erases a flash sector each
time. Suppress the *call* to `sendMetadata()`, never the transmission inside
it, or `metadataForced` is consumed and receivers lose PARM/UNIT/EQNS.

## Open: V2 board dies at the first beacon

See [docs/v2-beacon-crash.md](docs/v2-beacon-crash.md). Unresolved as of
2026-08-22, and it is in `main` and in the unpublished `v2.1.0` draft release.
Read the ruled-out list before proposing a cause - four hypotheses are already
eliminated with evidence.
