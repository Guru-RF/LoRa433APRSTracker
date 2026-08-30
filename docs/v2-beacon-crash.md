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

**Transmitting while USB-C is attached.** On the Powerpole alone the tracker
transmits at full drive and works: an iGate 8 m away decodes it at RSSI
-43 dBm, SNR 3.5. Unplug the Powerpole and transmit on USB-C and the PA ramp
stalls.

This is the failure the transmit inhibit exists to prevent. Every reproduction
during the 2026-08-30 bench session was produced by ejecting the drive to get
a serial console, which is what lifts the inhibit - so the whole session was
run in the one configuration the firmware is written to refuse.

The mechanism is a stalled PA ramp, not a reset: RadioLib waits for BUSY to
fall after SetTx **with no timeout of its own** (`src/radio.cpp`), so when the
ramp cannot complete the driver spins forever and the chip reports no error.

Both operating points are now verified against an iGate 8 m away:
`paDrive=22` on the Powerpole decodes at -42 dBm, and `usbPaDrive=8` with a
computer attached decodes at -42..-46 dBm. There is no silent band on the
Powerpole at rated drive; the cliff is a USB-power phenomenon only.

On USB the behaviour is drive-dependent, which is what made it look like a PA
configuration problem for a while:

| `paDrive` | supply           | result                                    |
|-----------|------------------|-------------------------------------------|
| 22        | Powerpole        | **works** - iGate decodes at -43 dBm      |
| 20        | Powerpole        | works                                     |
| 14        | USB-C            | works - low enough draw to stay inside it |
| 20        | USB-C            | stalls                                    |
| 22        | USB-C            | stalls                                    |

`loraOcp=60` also "works" on USB, for the same reason and no other: it clamps
the die below the drive it was asked for. It is not a usable setting - nothing
was heard on an iGate 8 m away with it.

## The firmware bug, now fixed

`sendMetadata()` used to run *before* `watchdog_enable(5000, true)`, so the
three telemetry frames every boot sends went out with no watchdog armed. A
BUSY stall there had no backstop and hung the board until someone unplugged
it - in a vehicle, silently and permanently. The same stall in `loop()` would
have been a 5 s reset.

`watchdog_enable()` now runs before the first transmission. This does not stop
the stall; it makes it survivable.

## Ruled out - do not re-derive

- **A supply rail collapsing.** Measured by the board owner from USB-C *and*
  from the Powerpole - no drop on either. Whatever USB-powered transmit does,
  it is not a rail sagging where a meter can see it. The
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

## Consequences for the firmware

The binary inhibit is the wrong response. Silence while a host is attached
means the bench setup can never be tested against an iGate, which is why this
took a whole evening to see. Scaling the drive instead - rated power on the
Powerpole, reduced drive while a host is enumerated - keeps the tracker on the
air in both cases and removes the need for the eject escape hatch entirely,
along with the `watchdog_hw->scratch[2]` latch that implements it.

Measured USB ceiling: **8**. Everything from -9 to 8 is decoded, with RSSI
tracking drive about 1:1 (-9 -> -56 dBm, 0 -> -50, 5 -> -45, 8 -> -46).
From 9 to 14 the tracker transmits cleanly and is never decoded - the
dangerous band, because it looks exactly like a working tracker. 17 takes the
USB link down and needs the cable physically pulled. usbPaDrive is clamped to
8 in the parser so the silent band cannot be selected.

Note RSSI saturates at this range: everything above drive 5 reads -42..-46 dBm
regardless, so at 8 m only "decoded or not" is a useful signal, not strength.

Still worth doing:

1. Read `GetDeviceErrors` (0x17) after a transmit in `TrackerRadio::send()`
   and report PA_RAMP / PLL_LOCK, so a stall says why instead of hanging.
2. Give the BUSY wait a bound so a stall degrades to `TX FAILED` rather than
   depending on a watchdog reset.
3. Note that the eject latch survives a watchdog reset, so a repeatedly
   stalling transmit currently retries forever. Removing the latch fixes this.
