// ============================================================
// RF.Guru LoRa 433 APRS Tracker - MeshCore advert
//
// Transmit-only participation in the IARU R1 ham MeshCore network
// (434.890 MHz, 62.5 kHz, SF8, CR4:8 - see the RFC at
// github.com/Guru-RF/meshcore-rfc-iaru-r1). The tracker emits a signed
// ADVERT carrying its position, alongside its normal APRS beacons.
//
// It does not receive. That is a deliberate scope limit, not an
// omission: everything a tracker needs to publish fits in an advert,
// and MeshCore's telemetry is request/response, so serving it would
// mean a full node - receive path, dedup, path handling and an ACL.
// Battery, temperature, humidity and satellite count stay on APRS,
// where they are already carried as typed telemetry.
//
// A repeater gates our position to APRS-IS over IP whether or not the
// advert is flooded (meshcore-repeater src/mesh.c gates before it
// decides on forwarding), so the default route is DIRECT with an empty
// path: heard by neighbours, gated, and not re-flooded across the mesh
// by a station that moves. meshRoute=flood restores normal behaviour.
// ============================================================

#pragma once

#include <Arduino.h>
#include "config_file.h"

namespace MeshCore {

// Load /meshid.bin, or create and persist a keypair if absent. Must run
// before FatFSUSB.begin(), while the filesystem is still ours. Returns
// false only if the identity could not be read or written.
bool begin(char *err, size_t errLen);

// True once an identity is available and meshEnabled is set.
bool ready();

// Node identity, for the boot report - the public key is the node's
// address on the mesh, so its first bytes are worth printing.
const uint8_t *publicKey();

// Build and transmit one signed advert at the given position. Retunes
// the radio to the mesh profile, transmits, and returns it to APRS.
// unixTime must be real (GPS-derived): MeshCore uses the advert
// timestamp for freshness, so a fabricated one poisons the mesh.
bool sendAdvert(const TrackerConfig &cfg, float lat, float lon, uint32_t unixTime);

}  // namespace MeshCore
