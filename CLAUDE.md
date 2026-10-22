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

`pio run -t uploadfs` does not work on this board and never has: the
platform's target builds a **LittleFS** image ("Building file system image
from 'data' directory to .pio/build/pico/littlefs.bin") regardless of
`board_build.filesystem = fatfs`, so it writes LittleFS into the FatFS
region. The firmware then finds no valid FAT, reformats, and writes a fresh
default `config.txt` - so uploadfs does not merely overwrite the owner's
callsign and settings, it discards `data/config.txt` too and leaves the
factory defaults. `/meshid.bin`, the station's MeshCore identity, goes with
it, and cannot be regenerated without changing its address on the mesh.
Provision `config.txt` the way the README does: copy it onto the mounted
drive and eject. A firmware `.uf2` writes only the sketch region, so ordinary
flashing leaves the filesystem alone. Back up both before anything that
touches the filesystem.

**Editing `config.txt` only takes effect if the drive is *ejected*.** Writing
it and then `diskutil unmount`-ing, or unmounting and then flashing, loses the
sector - the write never reaches flash. Eject: it commits and reboots.

**Watching the console takes a whole loop pass to observe anything.** A
stationary tracker beacons every 180 s and adverts every `meshInterval`, so a
capture shorter than about 7 minutes can easily contain zero beacons and prove
nothing. Beware too that the first beacon of a session always carries the
comment whatever `commentInterval` says, so it cannot confirm that setting -
only the second one can.

## Open, as of 2026-09-07

Two boards on the bench that day, and where each was left:

- **OR7F-5 (V2).** Its BME680 is dead (see "BME680"). Its boot burst loses
  the earliest frames whenever the module PA is amplifying, on either
  supply, and every firmware-side explanation has been closed (see the end
  of "V2: transmitting on USB power"). Its `config.txt` still carries the
  last experiment: `paDrive=8`, `usbPaDrive=-9`, `metaGap=3000`,
  `paWarmup=0`, `fullDebug=true`. Next: run the iGate with CRC-error
  logging (the `log_warn` in `../PILoRa433APRSiGate` `lora_take()`), then
  `paWarmup=2000` with `metaGap=150`.
- **OR7F-8 (V1).** Converted that day from the CircuitPython firmware in
  `5d1326b`; runs the generated default `config.txt` with `callsign=OR7F-8`,
  `comment=https://RF.Guru Q5`, `meshEnabled=true`, `meshName=OR7F-8`. It
  beaconed and sent the first MeshCore advert ever from an SX1276, then at
  18:02 dropped off USB and did not re-enumerate for minutes - not a
  watchdog reboot, which comes back in seconds; a pulled cable or a lost
  supply. It is still on the build *before* the reset-cause banner, so the
  first thing to do with it is flash HEAD and read the banner.
- The `usbPaDrive` clamp applies to V1 too, where it was never measured;
  the SX127x branch of `setDrive()` turns -9 into +2 dBm. Left as is,
  deliberately, until someone decides what a V1 on USB should do.

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

## First boot and the filesystem

`FatFS.begin()` formats the region when it finds no filesystem there, and
erasing 0.5 MB of flash takes longer than the 8 s watchdog allows. On a fresh
board - or one converted from the CircuitPython firmware in this repo's
history (`5d1326b`), whose FAT sits at a different offset - the watchdog
fired mid-format, the default `config.txt` written next was left with a
directory entry of size 0 over clusters full of text, and every later boot
hung reading it: milliseconds per byte, end of file never reached, watchdog
reset, repeat. A 12 s boot loop that prints only the first banner is this.

Three things now stand between a board and that loop, and each is
load-bearing: the watchdog is paused around `FatFS.begin()` (nothing else
runs there); `configCreateDefault()` feeds it through its ninety lines; and
`configLoad()` treats `read() < 0` as end of file - the old `char c =
f.read()` turned -1 into 0xFF and could not terminate - caps the byte count,
and gives the whole load 3 s, past which it reformats, rewrites the default
and reboots. That reformat discards `/meshid.bin` too; a board in a boot
loop has no identity worth more than booting.

The first banner carries the reset cause from `rp2040.getResetReason()` -
`power on`, `watchdog timeout`, `software reboot` (eject, `onUnplug`),
`RUN pin` - plus a count of restarts since the supply was last good, kept in
`watchdog_hw->scratch[3]`, which a watchdog or software reboot preserves and
a real power-on clears. The same text repeats in the 60 s "USB host still
connected" line with the uptime, because by the time anyone opens a console
on a board that restarted the banner is gone. On the RP2040 a brownout is
reported as `power on` (the chip has one POR flag for both), so a tracker
that keeps saying "power on" with a restart count of 0 is losing its supply,
and one that says "restart 7 since power-on" is losing to the firmware.
Uploading over a looping board is a race - picotool's 1200 baud touch misses
about half the time - so use BOOTSEL.

## BME680

`include/bme680_direct.h` drives the sensor through Bosch's `bme68x` API
directly rather than `Adafruit_BME680`. Two facts explain it, both
established on OR7F-5 with the board on the bench:

- **The sensor never worked before, on any unit.** `Adafruit_BME680 bme680;`
  defaults to `Wire` - I2C0, on GP4/GP5, which is the GPS UART here - and
  `begin(0x77, &Wire1)` compiled only because `begin()` has no `TwoWire`
  parameter: the pointer became `initSettings = true`. So there is no known
  good Adafruit configuration to fall back to.
- **`bme68x_init()` opens with a soft reset, and one of these parts does not
  come back from it.** It ACKs `0xE0 = 0xB6` and then NAKs everything: bus
  idle high, nothing stuck, no recovery after 610 ms, a 9-clock bus clear or
  an I2C block re-init. Only cutting its rail restores it - and only with SDA
  and SCL held low meanwhile, or the pull-ups back-power it through its ESD
  diodes and it never resets. Adafruit keeps `gas_sensor` private, so the
  reset cannot be skipped from outside; the write callback swallows it and
  `begin()` power-cycles the rail instead. Settings are Adafruit's verbatim.

Use `i2cRailOff()` to drop the sensor rail, never a bare
`digitalWrite(PIN_I2C_PWR, LOW)`, for the back-powering reason above.

**The part on OR7F-5 is beyond firmware.** With the reset out of the way it
initialises and reads its full calibration block, then vanishes the moment
a measurement starts - `read reg 0x1D code 4` - and comes back seconds later
in power-on state with its configuration gone. That holds with 1x
oversampling and the heater off, with GP3 at 12 mA drive, and with the bus
completely silent for 3 s after the trigger, so it is not load, not the rail
switch and not bus timing. A digital side that works while the measurement
engine collapses is a damaged or counterfeit part, or a module whose VDD
(not VDDIO) is unpowered; check both pins sit at 3.3 V and swap the module.
After three consecutive failed readings the beacon path gives up on it, cuts
the rail, and queues `BME680 gave up: rd 0x1D code 4` as an alert.

`fullDebug` prints the bus scan, the chip ID at 0x76/0x77, and either a
reading or the exact transfer that failed. The driver initialises the healthy
BME680 on OR7F-8 - the V1 board converted from CircuitPython, whose old
firmware read that same part - so the driver is not what is wrong with
OR7F-5's. A reading has still not been seen on a console (OR7F-8 came up
with `fullDebug` off), so the first unit that shows one should be checked
against a known thermometer before anyone trusts `bme680TempOffset`.

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
- **Group messages (`#mbox` alerts) are encrypt-then-MAC.** The repeater's
  `channel_build_txt()` encrypts in place and leaves the buffer named `plain`,
  so the `hmac_sha256()` call two lines later reads as though it covers the
  plaintext. It covers the ciphertext. Getting it backwards yields a packet
  the repeater receives, matches to the right channel, and drops with
  "failed public-channel MAC". The channel key is `SHA256("#name")[0..15]`,
  AES takes those 16 bytes, the HMAC takes them zero-padded to 32.
- Crypto for this is vendored in `lib/meshcrypto` **from the repeater tree**,
  not reimplemented, so the two cannot drift apart.

## MeshCom

`src/meshcom.cpp` is a second, unrelated mesh: transmit-only position
beacons and private messages on the MeshCom network (433.175, 250 kHz,
SF11, CR4:6). Unlike the MeshCore path it was written without a reference
tree to check against, and two of its settings were wrong on the air for
exactly that reason. Both are now verified against
`icssw-org/MeshCom-Firmware` (branch `dev`).

- **MeshCom does not use the 0x12 sync word.** It uses 0x2b:
  `SYNC_WORD_SX127x 0x2b` in `src/configuration_global.h`, and
  `Radio.SetCustomSyncWord(0x242b)` in `src/lora_setchip.cpp`. A node
  listening on 0x242b never sees a 0x12 preamble, so getting this wrong
  fails *silently* - the console prints TX MESHCOM, the frame is on the
  air, and the mesh is deaf to it. This is the only profile that changes
  the sync word, so `setMode()` restores 0x12 on the way back out; forget
  that and MeshCom silences APRS instead.
- **RadioLib cannot set 0x242b directly.** Its `SX126x::setSyncWord(sync,
  ctrl)` interleaves the two arguments, so the register pair comes from
  `sync=0x22, ctrl=0x4b`. `src/radio.cpp` carries the derivation - the
  numbers look like typos otherwise.
- **The preamble picks the country, and the country shows.** MeshCom's
  `DEFAULT_PREAMPLE_LENGTH` is 32, but that is the plain "EU" country
  profile; the ham networks on 433.175 run "EU8", whose entry in
  `lora_setcountry()` is labelled "EU Preabble 8" and sets
  `node_preamplebits = 8`. Nothing else differs between them. The tell is
  the modulation byte: upstream builds it as `(getMOD() & 0xF) |
  (node_country << 4)` and MHeard prints it as country/modulation, so a
  station on preamble 32 reads `0/8` in a list where every neighbour reads
  `8/8`. `src/meshcom.cpp` derives the country nibble from the configured
  preamble rather than carrying a second setting that could disagree with
  it. Preamble 8 also saves 24 symbols - 197 ms at SF11/250 kHz - on every
  frame.
- **The position's free text is `<comment>#<name>`,** and the `#` belongs
  to the name rather than separating two fields - `PositionToAPRS()` builds
  `cname` as `"#" + node_name`, so a node with no comment still transmits
  `#name`. That is why frames like `...31Ev#Anhaenger/B=100/...` appear on
  the air with the symbol immediately followed by a `#`. The comment is
  capped at 25 characters and `charset_filter` drops `{ } : ; , /` from it.
  A space survives that filter but still breaks the field, because
  `decodeAPRSPOS()` ends the free text at the first space - which is why
  every station in this network writes its comment with hyphens.
- **`meshComComment` is sent verbatim, and empty falls back to
  `Meshcom`.** Two earlier versions of this field did more and both were
  wrong. Inheriting the APRS `comment` when the key is empty looks tidy -
  one station, one description - but the two fields are not the same
  field: APRS allows 64 characters against this one's 25, and a URL, the
  commonest APRS comment there is, loses its `:` and `/` to
  `charset_filter` and arrives as `httpsphilipp.wagnersnet`. Forcing a
  `Meshcom-` prefix onto a comment the operator *did* set was the same
  mistake from the other side: both paths reach APRS-IS under the same
  callsign, so the text does have to say which network a position came out
  of - but someone who writes `vMesh` has already said it in five bytes,
  and the prefix would spend eight more restating it. The fallback is
  where that duty belongs, because an unset field is the only case where
  nobody has answered. `commentInterval=off` is the unrelated other half:
  it silences the APRS free text, telemetry and `/A=` excepted, for a
  station that would rather spend those bytes.
- **The firmware-version byte in the trailer is a protocol generation.**
  Three receivers read it: `aprs_functions.cpp:496` discards the entire
  frame when it is 1..34 ("Packet discarded, wrong FW-version"), and
  `lora_functions.cpp:763` and `loop_functions.cpp:2992` treat `/A=` as
  feet and convert it only when it is above 13. This firmware sent its own
  2.1 as `2`, which put it inside the discard window of every current node
  and left 541 m displaying as "1775" on the older ones that still
  accepted it. It now sends 35. Nodes built before that check went in
  accept anything, which is why a bench pair on February and April builds
  showed nothing wrong.

  What 35 *says* is "4.35": `shortVERSION()` is
  `memcpy(cfw, SOURCE_VERSION+2, 2)` over `SOURCE_VERSION "4.35"`, so the
  byte is only the part after the dot and the major version is never
  transmitted at all. 35 is therefore not a version this board picked but
  the network's current floor - the discard test is `< 35`, so anything
  lower is refused and anything higher claims a generation that does not
  exist yet. It has to be raised in step with upstream whenever that
  constant moves.

  The sub-version byte after the FCS is uninterpreted, and `'#'` is the
  network's own word for "not stated". Upstream sends
  `SOURCE_VERSION_SUB`, a letter (`"s"` in dev as of 2026-09), but nothing
  in the node firmware ever compares the received byte: it is printed
  (`printBuffer_aprs`, as `FW:%02i:%c`) and forwarded to the server as the
  JSON string `fw_sub` (`extudp_functions.cpp`), and that is all. So `'#'`
  is not this board inventing a marker - `decodeAPRS()` substitutes `'#'`
  itself whenever the byte is `0x00` or missing entirely (the trailer
  ending `0x7E` arrives in its place), and the encoder writes `0x23` for a
  zero field on the way out. Sending it says exactly "no sub-version",
  using the value every node already generates for that case, which is why
  it cannot trip a check that does not exist and cannot be mistaken for a
  release either. The corollary is that it does **not** identify this
  tracker: an old node and a truncated frame decode to the same `'#'`.
- **Two hop limits, not one.** `initAPRS()` gives `:` and `@` the text
  limit and everything else the position limit, which is why real nodes
  send positions at H02 and messages and HEY at H04. `meshComHopText`
  carries the second one.
- **HEY (`@`) is the diagnostic this board otherwise cannot have.** It
  asks who can hear the station, and every node that relays it appends its
  own RSSI/SNR before passing it on, so the answer accumulates inside the
  frame and lands in a gateway's RX log and at the server - reachable even
  though nothing here can receive. Upstream damps it with a trickle timer
  (`docs/hey-supp.md`: Imin 30 s, doubling to Imax 15 min, suppressed once
  two neighbours' HEYs are heard). Suppression needs a receiver, so this
  board sends at Imax and never runs the fast end of the ramp - a
  transmit-only station running trickle would be louder than any real
  node, not quieter.
- **The frame is verified against `extras/decode_meshcom.py`** in that
  tree, which is the cheapest available oracle: type, message id, hop
  byte, APRS-shaped text, then `[zero, hardware_id, lora_mod, fcs, fw,
  lasthw, fw_subver, ending]`. The FCS is a byte sum sent high byte
  first; that decoder swaps the halves before comparing, which is the
  same statement inverted.
- **A message is only shown by a node whose own callsign, group
  membership (`CheckOwnGroup`) or `*` matches the destination.** A node
  receiving a message for someone else still decodes and logs it, so
  "it arrives as TXT but not in the mailbox" means the frame is right
  and the *destination* is not. `*` is the way to prove the path.
- **The message id is a dedup key, not a counter.** A receiving node
  remembers the last MAX_DEDUP_RING ids it saw - 60..100 depending on
  the board - and drops a repeat *regardless of sender*
  (`dedup_functions.cpp`, `checkOwnRx`). Ids that restarted at 1 on every
  boot therefore had the first frames after each reflash silently
  discarded, which is most of what looked like "MeshCom is flaky". They
  now follow upstream: node identity hashed from the callsign in the top
  22 bits, a counter seeded from `rp2040.hwrand32()` in the low 10.
- **`meshComHardwareId=0` means "no info"**, not "unset" - it is index 0
  of the table in `mheard_functions.cpp`. The default is 1 (TLORA_V2),
  the closest plain SX127x node, because this board has no id of its own
  and a station reporting 0 may not reach the map. Ids above 38 are
  remapped by `getHardwareLong()`, which is why a T-Deck Plus reports 46
  and displays as index 17.
- **An alert needs no GPS.** It carries no position and the MeshCom path
  needs no timestamp, so the flush sits *above* the date gate in `loop()`,
  next to the forced-metadata send. Below it the queue was unreachable
  without a fix, which `alertFlush()`'s own comment already claimed it
  was not. This is also the only way to test the mesh path on a bench
  with no GPS antenna: every reboot puts one PM on the air.
- `meshComInterval=smart` hangs the position off the same SmartBeacon
  decision as APRS, with a 60 s floor. It is sent on the pass *after*
  the beacon, never the same one - the retune rule above applies here
  too.
- **Never set the track bit (0x40 in byte 5).** It was set for a while on
  every `smart` position, on the reasoning that upstream sets it for
  anything sent off the POSINFO interval. That reading is right about the
  encoder and wrong about what the flag *means* to the network. Track is
  a MeshCom node's declaration that its position goes out over LoRa-APRS
  instead of over the mesh, and `loop_functions.cpp:4453` is blunt about
  it: `bSendViaAPRS = bDisplayTrack; bSendViaMesh = !bDisplayTrack;` - a
  node in track mode stops feeding the mesh almost entirely. Nothing in
  the node firmware filters on the received bit (it appears only in
  `aprs_structures.h`, `aprs_functions.cpp` and `loop_functions.cpp`, and
  only to be encoded, decoded and printed), so the cost lands further
  upstream, at the server that decides what to gate to APRS-IS. The trap
  for this board in particular: the claim would be *true* - it really is
  on LoRa-APRS as well - and it still costs the station its MeshCom
  visibility for nothing in return.

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

### The boot-burst loss on OR7F-5, and what it is not

On the bench OR7F-5 loses the *earliest* frames of the three-frame metadata
burst whenever the module PA is amplifying, on either supply, and only the
last frame sent survives - the table is in
[docs/v2-beacon-crash.md](docs/v2-beacon-crash.md). A 34-agent review of the
iGate, RadioLib and this firmware, every claim adversarially verified, closed
the firmware-side explanations:

- **The iGate cannot lose a frame to the next one.** `../PILoRa433APRSiGate`
  is the C port: it polls RxDone every 10 ms, reads the FIFO and re-arms RX
  in one sub-millisecond pass, and has no dedup or rate limit. The air is
  silent for ~500 ms between our frames (100 ms post-TX hold, `metaGap`,
  250 ms PA pre-charge) plus a 262 ms preamble - not the 150 ms the gap key
  suggests.
- **Our three frames are byte-identical on the SPI bus**, each from STDBY_RC
  with a 200 us ramp; TX_DONE cannot be stale, frames cannot overlap, drive
  is frozen for the burst, LDRO and time-on-air are right.
- **The iGate logs nothing for a frame that fails CRC** - `rfm9x.c` only
  counts it - so "not in the journal" has never distinguished "arrived
  corrupt" from "never demodulated". That is the measurement that separates
  a dirty PA from a silent one; a `log_warn` in `lora_take()` for `len < 0`
  provides it.
- **`paDrive` 19..22 all program `SetTxParams(+22)`**; RadioLib's
  `paOptTable` varies only `paDutyCycle`/`hpMax` across them, so a ladder
  through those values moves the PA's operating point, not the die's power
  register. And 8 -> 9 jumps the power byte 13 -> 20, which is exactly where
  the USB silent band begins.
- The iGate's RSSI uses the HF-port offset in the LF band, so its numbers
  read about 7 dB high. Compare them with each other, not with a budget.

`metaGap` and `paWarmup` in `config.txt` exist for the two physical
hypotheses that remain; `metaGap=3000` has been run (still nothing) and
`paWarmup` has not. Both default to the behaviour deployed units have.
