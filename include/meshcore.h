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
// `drive` is the dBm to transmit at, chosen by the caller from the same
// USB-aware rule the APRS path uses. It is explicit rather than inherited:
// leaving it to whatever the last transmission set makes advert power
// depend on the order frames happen to go out, and on USB that silently
// lands in the band where the tracker transmits and nothing decodes it.
bool sendAdvert(const TrackerConfig &cfg, float lat, float lon,
                uint32_t unixTime, int8_t drive);

// Send one line of text to a MeshCore public channel, as a group message
// the way a phone client would. The channel key is derived from its name
// (SHA256("#mbox")[0..15]), so no key exchange is needed - anyone with the
// name can read it, which is the point on an amateur network where
// obscuring meaning is not allowed anyway.
//
// `drive` is chosen by the caller from the same rule the beacon uses.
// Returns false if MeshCore is not configured or the radio refused.
bool sendChannelText(const TrackerConfig &cfg, const char *channel,
                     const char *text, uint32_t unixTime, int8_t drive);

}  // namespace MeshCore
