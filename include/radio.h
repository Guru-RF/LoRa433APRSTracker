// ============================================================
// RF.Guru LoRa 433 APRS Tracker - Radio abstraction
//
// One firmware, two radios. The chip fitted to the board is
// identified over SPI at boot and the matching RadioLib driver is
// brought up with settings that put identical packets on the air:
//
//   V1 boards: HopeRF RFM95W / Semtech SX1276
//   V2 boards: G-NiceRF 1262MiniF27 (SX1262 + internal PA)
//
// Neither revision wires the radio interrupt line to the RP2040, so
// transmission is polled. The poll loop feeds the watchdog, which the
// old blocking LoRa.endPacket() could not do.
// ============================================================

#pragma once

#include <Arduino.h>
#include "config_file.h"

enum RadioChip {
    RADIO_CHIP_NONE = 0,
    RADIO_CHIP_SX127X,      // SX1276/77/78/79, RFM95/96/97/98
    RADIO_CHIP_SX126X       // SX1261/1262/1268
};

namespace TrackerRadio {

// Identify the fitted radio over raw SPI. Safe to call before begin();
// leaves the chip in reset-default state and emits no RF.
RadioChip probe();

// Bring up the detected radio. `cfg.radioChip` may pin the choice
// instead of auto-detecting. On failure returns false and fills `err`.
bool begin(const TrackerConfig &cfg, char *err, size_t errLen);

// Transmit one packet. Blocks until the chip reports TxDone (or the
// time-on-air budget expires), feeding the watchdog throughout.
bool send(const uint8_t *data, size_t len);

// Re-program the transmit drive between packets, clamped to what the
// fitted chip can produce. On a module with its own amplifier this is
// the drive into that amplifier; on a bare module it is the output
// power. Cheap and idempotent - a value already in force is a no-op -
// so callers may set it before every frame.
bool setDrive(int8_t dbm);

RadioChip   chip();
const char *chipName();

// True when the fitted module has an internal power amplifier, so the
// chip's own output is a *drive level* rather than the radiated power.
bool hasModulePa();

// Power actually programmed into the chip, after clamping. On a bare
// module this is the output power (SX1276 tops out at +20 dBm, SX1262
// at +22 dBm). On a module with its own amplifier it is the drive into
// that amplifier, and what reaches the antenna depends on VCC_PA and
// the filtering downstream - measure it, do not predict it.
int8_t appliedPower();

// TCXO supply voltage the SX126x was brought up with, 0.0 for a plain
// crystal module.
float tcxoVoltage();

// True when the SX126x die runs its internal LDO rather than the DC-DC,
// which is what a module without a DCC_SW inductor needs.
bool usingLdoRegulator();

// Expected time on air for a payload of `len` bytes, in milliseconds.
uint32_t timeOnAirMs(size_t len);

// Read the settings back out of the chip into a human-readable line,
// for confirming a new board revision is configured as intended.
void describe(char *buf, size_t len);

}  // namespace TrackerRadio
