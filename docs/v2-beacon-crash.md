# V2 board hangs at the first transmission

**Status:** root cause narrowed, workaround known, one firmware bug fixed.
Last updated 2026-08-30 from a bench session with a serial console attached.

## What actually happens

The tracker hangs inside `loraSendText()` on the first frame it tries to put
on the air. Everything before that is healthy: the radio is detected, the GPS
locks, the console runs for as long as you like - provided nothing transmits.

Two presentations, and they turn out to be the same fault at different drive
levels:

- **Hung, LEDs lit.** Power LED bright, LoRa LED stuck on, USB gone, dead
  until physically re-powered. The LoRa LED is the tell: `loraSendText()`
  raises it before keying and lowers it after, so a stuck LED puts the hang
  between those two lines.
- **Dark.** Reported from the field at full drive.

RP2040 pads latch their state through a hang, so lit LEDs prove the 3V3 rail
is up and the CPU has stopped - not a reset and not a power failure.

## Root cause

RadioLib waits for BUSY to fall after SetTx **with no timeout of its own**
(`src/radio.cpp`). When the SX1262 is asked for its maximum +22 dBm PA
configuration (`paDutyCycle=4, hpMax=7`) on this module, BUSY never falls and
the driver spins forever. The chip reports no error - it simply stops
answering.

It is the *top of the drive range* that fails, not the current drawn getting
there. Throttling by either available route avoids it:

| `paDrive` | `loraOcp` | result |
|-----------|-----------|--------------------------------------------------|
| 22        | 140       | hangs in `startTransmit`, LoRa LED stuck on      |
| 22        | 60        | works - the die's OCP clamps it below +22 dBm    |
| 14        | 140       | works - three metadata frames and beacons, stable |

`loraOcp=60` is not a usable setting: the die needs ~118 mA to make +22 dBm,
so clamping to 60 buys survival by throttling the transmitter below useful
output. Nothing was heard on an iGate 8 m away.

**Working setting today: `paDrive=14`.** Note this contradicts commit bfe48bf,
which set `paDrive=22` deliberately because that is what the module needs for
its rated output. The board therefore cannot currently reach rated power.

## The firmware bug, now fixed

`sendMetadata()` used to run *before* `watchdog_enable(5000, true)`, so the
three telemetry frames every boot sends went out with no watchdog armed. A
BUSY stall there had no backstop and hung the board until someone unplugged
it - in a vehicle, silently and permanently. The same stall in `loop()` would
have been a 5 s reset.

`watchdog_enable()` now runs before the first transmission. This does not stop
the stall; it makes it survivable.

## Ruled out - do not re-derive

- **Supply, current, and voltage drop.** Measured by the board owner at
  another location, from USB-C *and* from the Powerpole. Not a brownout. The
  3V3 LDO arithmetic (XC6206P332MR, 200 mA, versus ~118 mA for the die at
  +22 dBm) looks alarming on paper and is *not* what is happening.
- **TCXO / oscillator.** The MiniF27 datasheet block diagram shows a plain
  32 MHz crystal and no DIO3 supply path; DIO3 is brought out to the host as a
  free pin. `probeTcxoVoltage()` printing `clock: crystal` is correct. Forcing
  `loraTcxo=1.8` appeared to fix it once and did not - that was an intermittent
  fault reproducing intermittently.
- **Time on air.** It dies mid-first-frame: ~2.0 s elapsed against 2.55 s
  needed to complete the shortest metadata frame (250 ms PA settle + 2302 ms
  at SF12/BW125/CR4:5). It never finishes a frame, so frame length is
  irrelevant and shortening `comment=` proves nothing - `comment=` does not
  appear in a metadata frame at all.
- **GP2 / PA enable.** `GP2` is not connected to anything on the V2
  schematic. The MiniF27 has no enable pin; its antenna switch is driven
  internally from DIO2, which never leaves the module. `loraSendText()`
  asserting GP2 and waiting 250 ms is dead code on V2.
- **I2C sensor init.** It died there once, before `sendMetadata()`, and walked
  straight through on every subsequent boot. That was a red herring.
- **The missing `-u _printf_float`.** Real, and fixed, but runtime-benign:
  varargs stay aligned, and config parsing uses `atof()`, never `scanf`.

## Still open

Why BUSY never falls at +22 dBm on this module, given the supply is sound.
Worth trying next:

1. `tools/chipprobe -e radiotest-tx`, which sends three frames at die drive
   0, 10 and 22 dBm in one boot and prints the SX126x device-errors word after
   each. That is the instrumented version of the sweep done here by hand.
   Change `DEFAULT_CALLSIGN` before building or it transmits as ON9RFG.
2. Bisect `paDrive` between 14 and 22 to find the exact threshold.
3. Read `GetDeviceErrors` (0x17) after a transmit in `TrackerRadio::send()`
   and report PA_RAMP / PLL_LOCK, so the firmware says why rather than hanging.
4. Give the BUSY wait a bound so a stall degrades to `TX FAILED` instead of
   relying on a watchdog reset.
