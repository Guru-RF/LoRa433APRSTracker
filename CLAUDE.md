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
the owner's callsign and settings - and `/meshid.bin`, the station's MeshCore
identity, which cannot be regenerated without changing its address on the
mesh. A firmware `.uf2` writes only the sketch region, so ordinary flashing
leaves the filesystem alone. Back up both before anything that touches the
filesystem.

**Editing `config.txt` only takes effect if the drive is *ejected*.** Writing
it and then `diskutil unmount`-ing, or unmounting and then flashing, loses the
sector - the write never reaches flash. Eject: it commits and reboots.

**Watching the console takes a whole loop pass to observe anything.** A
stationary tracker beacons every 180 s and adverts every `meshInterval`, so a
capture shorter than about 7 minutes can easily contain zero beacons and prove
nothing. Beware too that the first beacon of a session always carries the
comment whatever `commentInterval` says, so it cannot confirm that setting -
only the second one can.

## Things that look like bugs but are not

- **GP15 is not a board-revision strap.** It measures low on V1 hardware too,
  so it identifies nothing. The revision comes from the radio, read over SPI
  in `src/radio.cpp`. `include/pins.h` keeps it documented and commented out
  so nobody wires it into a decision again.
- **The radio interrupt line is not routed** on either revision, so transmit
  is polled rather than interrupt-driven. The poll loop feeds the watchdog,
  which the old blocking `LoRa.endPacket()` could not.
- **RadioLib waits on BUSY with no timeout** after SetTx, and the SX1262
  does it for real on V2 at `paDrive=22` - the driver spins forever and the
  chip reports no error. The 5 s watchdog is the only backstop, so it is now
  armed *before* the boot metadata burst rather than after; it used to be
  enabled last, which left every boot's first three frames unprotected and
  turned a stall into a board that was dead until unplugged.
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

Rather than refusing to transmit, the drive is scaled: `usbPaDrive` while a
host is enumerated, rated `paDrive` otherwise, chosen per frame so it follows
the cable with no reboot. `usbTxInhibit=true` restores full silence for a port
that cannot supply even the floor drive.

That reduced-drive state announces itself - a console line every 60 s and a
power-LED wink every 5 s - because a car head unit enumerates USB mass storage
and would otherwise hold a tracker at -9 dBm for a whole journey with
well-formed frames that nothing decodes.

The eject latch in `watchdog_hw->scratch[2]` that an earlier design used is
gone; ejecting now only reboots to apply the config, as the README says.

The gate in `loop()` must stay **above** `sb.shouldBeacon()`. Suppressing
between that call and `sb.updateAfterBeacon()` leaves `_lastLat` at 999, which
makes `shouldBeacon()` true on every 50 ms pass and erases a flash sector each
time. Suppress the *call* to `sendMetadata()`, never the transmission inside
it, or `metadataForced` is consumed and receivers lose PARM/UNIT/EQNS.

## APRS airtime

At SF12 an 82-byte frame is 3.45 s on air, so anything that shortens it or
sends it less often matters far more than on 1200 baud AX.25. Two
non-obvious facts, both from primary sources:

- **The payload quantises in 5-byte, 164 ms steps.** Trimming fewer than
  five bytes saves nothing at all.
- **Altitude is free while stationary.** APRS 1.0.1 chapter 9 page 38: when
  the compressed position's `c` byte is a space, "the csT bytes are ignored".
  Page 40: with the T byte's NMEA-source bits set to GGA (bits 4,3 = 10),
  `cs` carries altitude = 1.002^cs feet. So a parked tracker can put
  altitude in three bytes it was already wasting, instead of nine more for
  `/A=`. Course/speed and altitude are mutually exclusive in that field;
  moving, course/speed wins.
- **aprs.fi caches the comment.** Its author: the comment "will be forgotten
  if you still transmit packets without a comment after 7 days". So
  `commentInterval` is safe on aprs.fi. It is seconds, not a beacon count, and
  defaults to 1800.
- Compressed reports allow **40 comment characters**, not the 43 of an
  uncompressed one.

## MeshCore adverts

`src/meshcore.cpp` puts the tracker on the IARU R1 ham MeshCore channel
(434.890 MHz, 62.5 kHz, SF8, CR4:8) alongside APRS, transmit-only. Wire
formats are verified against `../meshcore-repeater` (`src/packet.h`,
`src/advert.h`), itself verified against upstream MeshCore.

- **One radio, two profiles.** `TrackerRadio::setMode()` retunes between them.
  They share only the sync word - MeshCore's default is RadioLib's 0x12
  expanded to 0x1424, the same value APRS uses - so frequency, bandwidth,
  spreading factor, coding rate and preamble all change, and LDRO flips
  because SF12/125 is a 32.8 ms symbol and SF8/62.5 is 4.1 ms.
- **An advert never shares a pass with an APRS beacon**, metadata or an alert.
  A retune must not land between another transmission's frames, and yielding
  costs one 50 ms loop pass. The check must sit *above* the "nothing to do"
  early return in `loop()` - below it the advert is unreachable, which is how
  it was first written and why it never fired.
- **Route is DIRECT by default, not FLOOD.** A repeater gates a position to
  APRS-IS *before* it decides whether to forward (`meshcore-repeater`
  `src/mesh.c`), so direct still reaches APRS-IS without a moving station
  re-flooding the mesh. `meshRoute=flood` restores normal behaviour.
- **`meshNodeType` must be `chat` or `repeater`** to reach APRS-IS;
  `aprsis_gate_node()` ignores `sensor` and `room`.
- **No advert until GPS time is valid.** MeshCore uses the timestamp for
  freshness and this board has no RTC, so there is no mesh presence indoors.
  That is deliberate, not a gap.
- **Telemetry stays on APRS.** MeshCore telemetry is request/response
  CayenneLPP, so serving it needs a receive path, dedup, paths and an ACL - a
  whole node. The advert's `app_data` is 32 bytes with no telemetry field, so
  it cannot be pushed either.
- Ed25519 signing measured at **39 ms** on this board (orlp/ed25519, the same
  library MeshCore vendors), so it needs no watchdog special-casing.

## V2: transmitting on USB power

Resolved: **transmitting while USB-C is attached** stalls the PA ramp. On the
Powerpole the tracker runs at rated drive and an iGate 8 m away decodes it at
-43 dBm. See [docs/v2-beacon-crash.md](docs/v2-beacon-crash.md), which also
records what was wrongly blamed first - supply current, the oscillator, time on
air, GP2 - so nobody re-derives them.

`usbPaDrive` is clamped to **8**, set by what an iGate actually decodes rather
than by what transmits without hanging: 9..14 transmit cleanly and are never
heard, and 17 takes the USB link down. An earlier note here recommended
`paDrive=14`, which is both the wrong key - `currentDrive()` ignores `paDrive`
entirely while a host is enumerated - and a value in the silent band.
