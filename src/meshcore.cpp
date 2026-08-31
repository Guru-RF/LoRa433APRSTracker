// ============================================================
// RF.Guru LoRa 433 APRS Tracker - MeshCore advert
//
// Wire formats verified against meshcore-repeater (src/packet.h,
// src/advert.h), which is itself verified against upstream MeshCore.
// ============================================================

#include "meshcore.h"

#include <FatFS.h>
#include <hardware/watchdog.h>

#include "pins.h"
#include "radio.h"

extern "C" {
#include "ed_25519.h"
#include "aes128.h"
#include "sha256.h"
#include "hmac_sha256.h"
}

// ============================================================
// Wire format
// ============================================================

// Packet header, 0bVVPPPPRR: version, payload type, route type.
static const uint8_t PH_ROUTE_FLOOD        = 0x01;
static const uint8_t PH_ROUTE_DIRECT       = 0x02;
static const uint8_t PH_TYPE_ADVERT        = 0x04;
static const uint8_t PH_TYPE_GRP_TXT       = 0x05;
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
        // Remove the stub rather than leaving it to be read back as a
        // corrupt file on every later boot - which would report an operator
        // error for a file this code wrote five lines ago and knows is bad,
        // and would disable MeshCore permanently.
        FatFS.remove(IDENTITY_PATH);
        snprintf(err, errLen, "short write creating %s - removed", IDENTITY_PATH);
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
                          uint32_t unixTime, int8_t drive) {
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
    // Whatever happens, hand the radio back on the APRS profile. Nothing on
    // the APRS path re-asserts it, so a mesh retune that bailed early and
    // returned without restoring would leave every later beacon, alert and
    // metadata frame transmitting on 434.890 - unheard by the iGate, and a
    // multi-second SF12 blast across the MeshCore channel - until the
    // tracker was power-cycled.
    char err[64];
    bool ok = false;
    if (TrackerRadio::setMode(RADIO_MODE_MESH, cfg, err, sizeof(err))) {
        TrackerRadio::setDrive(drive);
        // Bracket the transmission with the amplifier enable exactly as
        // loraSendText() does. GP2 is unconnected on V2 so this changes
        // nothing there, but on a V1 board it gates the 13.8 V amplifier
        // and an advert would otherwise go out unamplified.
        bool needPa = cfg.hasPa || TrackerRadio::hasModulePa();
        if (needPa) { digitalWrite(PIN_PA, HIGH); delay(250); watchdog_update(); }
        ok = TrackerRadio::send(frame, 2 + payLen);
        if (needPa) { delay(100); digitalWrite(PIN_PA, LOW); }
    }
    if (!TrackerRadio::setMode(RADIO_MODE_APRS, cfg, err, sizeof(err))) {
        // The radio is now on an unknown profile and APRS is about to use
        // it. Say so loudly; setMode has left its cache invalid, so the
        // next attempt reprograms from scratch rather than trusting it.
        Serial.printf("\x1b[1;5;31mRADIO STUCK OFF-PROFILE: %s\x1b[0m\r\n", err);
    }
    return ok;
}

// ============================================================
// Public channel group text
//
// Wire format taken from channel_build_txt() in the repeater's
// src/mesh.c, so what this encrypts is decrypted by exactly the code that
// receives it:
//
//   key      = SHA256("#name")[0..15]
//   selector = SHA256(key, 16)[0]
//   plain    = [ts u32 LE][txt_type 0]["sender: text"], zero-padded to 16
//   cipher   = AES-128-ECB(key, plain)
//   payload  = [selector][HMAC-SHA256(key32, cipher)[0..1]][cipher]
//
// The key asymmetry is deliberate and easy to get wrong: AES takes the
// 16-byte key, the HMAC takes it zero-padded to 32.
// ============================================================

bool MeshCore::sendChannelText(const TrackerConfig &cfg, const char *channel,
                               const char *text, uint32_t unixTime, int8_t drive) {
    if (!identityLoaded || !channel || !channel[0] || !text || !text[0]) return false;

    uint8_t key32[32];
    memset(key32, 0, sizeof(key32));
    SHA256_CTX c;
    uint8_t digest[SHA256_BLOCK_SIZE];
    sha256_init(&c);
    sha256_update(&c, (const uint8_t *)channel, strlen(channel));
    sha256_final(&c, digest);
    memcpy(key32, digest, 16);                 // channel key = SHA256(name)[0..15]

    sha256_init(&c);
    sha256_update(&c, key32, 16);              // selector hashes the raw 16, not the pad
    sha256_final(&c, digest);
    uint8_t selector = digest[0];

    uint8_t plain[160];
    size_t i = 0;
    memcpy(plain, &unixTime, 4); i = 4;
    plain[i++] = 0;                            // txt_type = plain
    int n = snprintf((char *)&plain[i], sizeof(plain) - i - 16, "%s: %s",
                     cfg.meshName, text);
    if (n < 0) return false;
    i += (size_t)n;                            // no NUL, matching MeshCore
    size_t ctlen = (i + 15) & ~(size_t)15;
    if (ctlen == 0) ctlen = 16;
    while (i < ctlen) plain[i++] = 0;

    // Encrypt first, then MAC the CIPHERTEXT. channel_build_txt() encrypts
    // in place and only then calls hmac_sha256() on the same buffer, which
    // reads like a MAC over the plaintext and is not - getting this
    // backwards produces a packet the repeater receives, matches to the
    // right channel, and drops with "failed public-channel MAC".
    aes128_ctx_t ac;
    aes128_init(&ac, key32);
    aes128_ecb_encrypt(&ac, plain, ctlen);

    uint8_t mac[32];
    hmac_sha256(key32, sizeof(key32), plain, ctlen, mac);

    uint8_t route = (strcasecmp(cfg.meshRoute, "flood") == 0) ? PH_ROUTE_FLOOD
                                                              : PH_ROUTE_DIRECT;
    uint8_t frame[2 + 3 + sizeof(plain)];
    frame[0] = (uint8_t)(route | (PH_TYPE_GRP_TXT << PH_TYPE_SHIFT));
    frame[1] = 0x00;                           // empty path
    frame[2] = selector;
    frame[3] = mac[0];
    frame[4] = mac[1];
    memcpy(frame + 5, plain, ctlen);

    char err[64];
    bool ok = false;
    if (TrackerRadio::setMode(RADIO_MODE_MESH, cfg, err, sizeof(err))) {
        TrackerRadio::setDrive(drive);
        bool needPa = cfg.hasPa || TrackerRadio::hasModulePa();
        if (needPa) { digitalWrite(PIN_PA, HIGH); delay(250); watchdog_update(); }
        ok = TrackerRadio::send(frame, 5 + ctlen);
        if (needPa) { delay(100); digitalWrite(PIN_PA, LOW); }
    }
    if (!TrackerRadio::setMode(RADIO_MODE_APRS, cfg, err, sizeof(err))) {
        Serial.printf("\x1b[1;5;31mRADIO STUCK OFF-PROFILE: %s\x1b[0m\r\n", err);
    }
    return ok;
}
