# V2 board goes dark at the first beacon

**Status:** open as of 2026-08-22. Present in `main` and in the unpublished
`v2.1.0` draft release.

## Symptom

V2 board (SX1262 + ATGM336H), powered from the 13.8 V Powerpole with no USB
attached. The tracker boots normally, the power LED lights, the GPS LED blinks
while acquiring - and the moment the GPS LED goes steady, all three LEDs go
dark and **stay** dark until the tracker is unplugged.

With the GPS disconnected it runs indefinitely, probing for a receiver. So the
fault is reached only after a fix.

The first fix triggers a beacon immediately: SmartBeacon's `_lastLat` stays at
999 until the first `updateAfterBeacon()`, so `shouldBeacon()` returns true
unconditionally on the first pass with a fix (`include/smartbeacon.h`). The
first transmission after lock is therefore what kills it.

## Ruled out - do not re-derive

- **The USB transmit inhibit.** Powerpole-only never enumerates, so
  `tud_mounted()` is false, `usbHostSeen` is never set and the gate is inert.
- **A firmware hang.** A hung CPU leaves GPIOs latched, so `PIN_LED_PWR` -
  driven high once in `setup()` and never touched again - would stay lit.
- **A watchdog reset.** It re-runs `setup()` and relights `PIN_LED_PWR` within
  microseconds. The owner confirmed it stays dark, so nothing is rebooting.
- **PA drive current on its own.** The boot metadata already transmits three
  frames at full power and survives. `paDrive` is not the trigger.

Between them these eliminate software as the direct cause: firmware cannot
turn its own power LED off while running, and nothing is resetting.

## Leading hypothesis

Time on air rather than peak current. The boot metadata is ~60 B, ~2.6 s on
air; a position beacon is ~95 B, ~3.7 s - roughly 40% longer at the same draw.
A marginal supply survives the short transmission and not the long one.

Once it browns out mid-transmit, GP2 (`PIN_PA`) goes hi-Z, because it is only
driven low inside `setup()`. With no pull-down on the PA enable, the amplifier
stays keyed and holds the rail down, so the board never gets far enough to
boot. That matches "dark, and stays dark until unplugged" exactly.

## Next tests

1. **Config only, no reflash:** shorten `comment=` in `config.txt` to a couple
   of characters. That cuts payload and time on air and changes nothing else.
   Surviving with a short comment and dying with a long one confirms a
   duration/current threshold.
2. **Meter or scope on the rail during transmit.**
3. **Check whether GP2 has a pull-down** in the schematic.

## Caveat on the regression window

The owner reports that "the previous release worked fine", but that was most
likely a V1 board. `v2.0.1` is built on `LoRa.h`, which cannot drive an SX1262
at all, so it is not a usable control on V2 hardware. This may be a V2
bring-up problem rather than a regression from the V1/V2 firmware work.
