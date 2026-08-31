// ============================================================
// RF.Guru LoRa 433 APRS Tracker - MeshCore advert
//
// Wire formats verified against meshcore-repeater (src/packet.h,
// src/advert.h), which is itself verified against upstream MeshCore.
// ============================================================

#include "meshcore.h"

#include <FatFS.h>
#include <hardware/watchdog.h>

#include "radio.h"

extern "C" {
#include "ed_25519.h"
}

// ============================================================
// Wire format
// ============================================================

// Packet header, 0bVVPPPPRR: version, payload type, route type.
static const uint8_t PH_ROUTE_FLOOD        = 0x01;
static const uint8_t PH_ROUTE_DIRECT       = 0x02;
static const uint8_t PH_TYPE_ADVERT        = 0x04;
static const uint8_t PH_TYPE_SHIFT         = 2;

// app_data flag bits. Only LATLON and NAME are used - feat1/feat2 are
// for node capabilities we do not claim.
static const uint8_t ADV_LATLON_MASK       = 0x10;
static const uint8_t ADV_NAME_MASK         = 0x80;

static const size_t  MC_PUB_KEY_SIZE       = 32;
static const size_t  MC_PRV_KEY_SIZE       = 64;
static const size_t  MC_SIGNATURE_SIZE     = 64;
static const size_t  MC_MAX_ADVERT_DATA    = 32;

// app_data is capped at 32 bytes and the flags, latitude and longitude
// account for nine of them. That is the real limit on meshName.
static const size_t  MC_MAX_NAME_LEN       = MC_MAX_ADVERT_DATA - 1 - 8;

// Identity file. A magic and version so a truncated or foreign file is
// rejected rather than used as a key.
static const char    *IDENTITY_PATH        = "/meshid.bin";
static const uint32_t IDENTITY_MAGIC       = 0x4D435F31u;   // "MC_1"

struct IdentityFile {
    uint32_t magic;
    uint8_t  pub[MC_PUB_KEY_SIZE];
    uint8_t  prv[MC_PRV_KEY_SIZE];
};

static IdentityFile identity;
static bool         identityLoaded = false;

// ============================================================
// Identity
// ============================================================

// Seed from the RP2040's ring oscillator, never from millis(). A
// predictable seed is a forgeable identity: the private key is the only
// thing stopping anyone advertising as this station.
static void seedFromHardware(uint8_t seed[32]) {
    for (int i = 0; i < 8; i++) {
        uint32_t r = rp2040.hwrand32();
        memcpy(seed + i * 4, &r, 4);
    }
}

bool MeshCore::begin(char *err, size_t errLen) {
    File f = FatFS.open(IDENTITY_PATH, "r");
    if (f) {
        size_t n = f.read((uint8_t *)&identity, sizeof(identity));
        f.close();
        if (n == sizeof(identity) && identity.magic == IDENTITY_MAGIC) {
            identityLoaded = true;
            return true;
        }
        // Present but unusable. Refuse rather than silently minting a new
        // identity over it - the operator may have restored a backup badly,
        // and a node that changes address on its own is worse than a node
        // that says why it will not start.
        snprintf(err, errLen, "%s is corrupt - delete it to mint a new identity",
                 IDENTITY_PATH);
        return false;
    }

    uint8_t seed[32];
    seedFromHardware(seed);
    ed25519_create_keypair(identity.pub, identity.prv, seed);
    identity.magic = IDENTITY_MAGIC;

    f = FatFS.open(IDENTITY_PATH, "w");
    if (!f) {
        snprintf(err, errLen, "could not create %s", IDENTITY_PATH);
        return false;
    }
    size_t n = f.write((const uint8_t *)&identity, sizeof(identity));
    f.close();
    if (n != sizeof(identity)) {
        snprintf(err, errLen, "short write creating %s", IDENTITY_PATH);
        return false;
    }

    identityLoaded = true;
    return true;
}

bool MeshCore::ready() { return identityLoaded; }

const uint8_t *MeshCore::publicKey() { return identity.pub; }

// ============================================================
// Advert
// ============================================================

bool MeshCore::sendAdvert(const TrackerConfig &cfg, float lat, float lon,
                          uint32_t unixTime) {
    if (!identityLoaded) return false;

    // ---- app_data: flags, position, name ----
    uint8_t appData[MC_MAX_ADVERT_DATA];
    size_t  appLen = 0;

    uint8_t nodeType = 1;                                   // ADV_TYPE_CHAT
    if (strcasecmp(cfg.meshNodeType, "repeater") == 0)      nodeType = 2;
    else if (strcasecmp(cfg.meshNodeType, "room") == 0)     nodeType = 3;
    else if (strcasecmp(cfg.meshNodeType, "sensor") == 0)   nodeType = 4;

    size_t nameLen = strlen(cfg.meshName);
    if (nameLen > MC_MAX_NAME_LEN) nameLen = MC_MAX_NAME_LEN;

    appData[appLen++] = (uint8_t)(nodeType | ADV_LATLON_MASK |
                                  (nameLen ? ADV_NAME_MASK : 0));

    int32_t latE6 = (int32_t)lroundf(lat * 1000000.0f);
    int32_t lonE6 = (int32_t)lroundf(lon * 1000000.0f);
    memcpy(appData + appLen, &latE6, 4); appLen += 4;
    memcpy(appData + appLen, &lonE6, 4); appLen += 4;
    memcpy(appData + appLen, cfg.meshName, nameLen); appLen += nameLen;

    // ---- signature over pub_key || timestamp || app_data ----
    uint8_t signed_[MC_PUB_KEY_SIZE + 4 + MC_MAX_ADVERT_DATA];
    size_t  signedLen = 0;
    memcpy(signed_, identity.pub, MC_PUB_KEY_SIZE); signedLen += MC_PUB_KEY_SIZE;
    memcpy(signed_ + signedLen, &unixTime, 4);      signedLen += 4;
    memcpy(signed_ + signedLen, appData, appLen);   signedLen += appLen;

    uint8_t sig[MC_SIGNATURE_SIZE];
    watchdog_update();
    ed25519_sign(sig, signed_, signedLen, identity.pub, identity.prv);
    watchdog_update();

    // ---- payload: pub_key | timestamp | signature | app_data ----
    uint8_t payload[MC_PUB_KEY_SIZE + 4 + MC_SIGNATURE_SIZE + MC_MAX_ADVERT_DATA];
    size_t  payLen = 0;
    memcpy(payload, identity.pub, MC_PUB_KEY_SIZE);   payLen += MC_PUB_KEY_SIZE;
    memcpy(payload + payLen, &unixTime, 4);           payLen += 4;
    memcpy(payload + payLen, sig, MC_SIGNATURE_SIZE); payLen += MC_SIGNATURE_SIZE;
    memcpy(payload + payLen, appData, appLen);        payLen += appLen;

    // ---- frame: header | path_len | payload ----
    //
    // path_len is 0: no hops recorded, hash size 1. On a DIRECT packet
    // that leaves no next hop, so neighbours gate it and nothing
    // re-transmits it - which is the point. On FLOOD it is the normal
    // starting state and each hop appends its own hash.
    uint8_t route = (strcasecmp(cfg.meshRoute, "flood") == 0) ? PH_ROUTE_FLOOD
                                                              : PH_ROUTE_DIRECT;
    uint8_t frame[2 + sizeof(payload)];
    frame[0] = (uint8_t)(route | (PH_TYPE_ADVERT << PH_TYPE_SHIFT));
    frame[1] = 0x00;
    memcpy(frame + 2, payload, payLen);

    // ---- retune, transmit, and always come back to APRS ----
    char err[64];
    if (!TrackerRadio::setMode(RADIO_MODE_MESH, cfg, err, sizeof(err))) return false;
    bool ok = TrackerRadio::send(frame, 2 + payLen);
    TrackerRadio::setMode(RADIO_MODE_APRS, cfg, err, sizeof(err));
    return ok;
}
