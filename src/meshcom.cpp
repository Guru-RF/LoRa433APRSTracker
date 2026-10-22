// ============================================================
// RF.Guru LoRa 433 APRS Tracker - MeshCom position beacon
// ============================================================

#include "meshcom.h"

#include <hardware/watchdog.h>
#include <math.h>
#include <string.h>

#include "pins.h"
#include "radio.h"

namespace {

// The message id is not a free counter. A receiving node remembers the last
// MAX_DEDUP_RING ids it has seen (60..100 depending on the board, see
// dedup_functions.cpp upstream) and silently discards a frame whose id is
// already in that ring - regardless of who sent it. A tracker that starts
// counting at 1 on every boot therefore has its first frames after a reboot
// dropped by any node that was listening before it, with nothing in either
// log to say so.
//
// Upstream splits the 32 bits the way loop_functions.cpp does:
// ((_GW_ID & 0x3FFFFF) << 10) | (node_msgid & 0x3FF) - the node's own
// identity in the top 22 bits, a rolling counter in the bottom 10. This does
// the same, with the identity hashed from the callsign so two trackers never
// share a space, and the counter seeded per boot from the hardware RNG so a
// restart does not replay ids that are still in a neighbour's ring.
static uint32_t meshComNodeId = 0;
static uint16_t meshComCounter = 0;

static uint32_t meshComNextMsgId() {
    uint32_t id = ((meshComNodeId & 0x3FFFFFUL) << 10) | (meshComCounter & 0x3FF);
    meshComCounter = (uint16_t)((meshComCounter + 1) & 0x3FF);
    return id;
}

// The modulation byte is two nibbles, not one: upstream builds it as
// (getMOD() & 0xF) | (node_country << 4), and every node's MHeard entry
// prints it back as country/modulation - "8/8" across this network. The
// low nibble mirrors getMOD() exactly.
static uint8_t meshComModulation(const TrackerConfig &cfg) {
    const float bw = cfg.meshComBandwidth;
    const int sf = cfg.meshComSf;
    const int cr = cfg.meshComCr;

    uint8_t mod = 3;  // medium
    if (sf == 12 && cr == 8 && fabsf(bw - 125.0f) < 0.01f) mod = 4;
    if (sf == 12 && cr == 6 && fabsf(bw - 125.0f) < 0.01f) mod = 5;
    if (sf == 10 && cr == 6 && fabsf(bw - 125.0f) < 0.01f) mod = 6;
    if (sf == 11 && cr == 5 && fabsf(bw - 250.0f) < 0.01f) mod = 7;
    if (sf == 11 && cr == 6 && fabsf(bw - 250.0f) < 0.01f) mod = 8;

    // Country, derived rather than configured. On this band EU (0) and EU8
    // (8) share frequency, bandwidth, spreading factor and coding rate and
    // differ only in the preamble, so the preamble in use *is* the country
    // - see lora_setcountry() cases 0 and 8. Deriving it keeps the byte
    // honest if the preamble is ever changed back.
    const uint8_t country = (cfg.meshComPreamble == 8) ? 8 : 0;
    return (uint8_t)((country << 4) | (mod & 0x0F));
}

static bool meshComPayload(char *out, size_t outLen, const TrackerConfig &cfg,
                           float lat, float lon, float altMeters,
                           int battPercent) {
    if (lat < -90.0f || lat > 90.0f || lon < -180.0f || lon > 180.0f) return false;

    char symTable = '/';
    char symCode = '#';
    if (strlen(cfg.symbol) >= 2) {
        symTable = cfg.symbol[0];
        symCode = cfg.symbol[1];
    }
    // MeshCom knows only the primary and secondary symbol tables - its own
    // command says so, "--symid set prim/sec Sym-Table" - so an APRS overlay
    // character has no meaning to it. This board ships with the L overlay
    // that LoRa trackers conventionally use on APRS, and every real node on
    // the mesh puts a plain '/' in that position. Sending L there hands
    // every receiver a table it does not have, so fold an overlay back to
    // the primary table for this network only; the APRS beacon keeps it.
    if (symTable != '/' && symTable != '\\') symTable = '/';

    char latHem = lat >= 0 ? 'N' : 'S';
    char lonHem = lon >= 0 ? 'E' : 'W';
    float absLat = fabsf(lat);
    float absLon = fabsf(lon);
    int latDeg = (int)absLat;
    int lonDeg = (int)absLon;
    float latMin = (absLat - latDeg) * 60.0f;
    float lonMin = (absLon - lonDeg) * 60.0f;
    float latAprs = latDeg * 100.0f + latMin;
    float lonAprs = lonDeg * 100.0f + lonMin;

    int altFeet = (int)lroundf(altMeters * 3.2808399f);
    if (altFeet < 0) altFeet = 0;

    // The free-text segment, exactly as PositionToAPRS() assembles it:
    // the comment, then the name behind a '#'. The '#' belongs to the name
    // rather than separating two fields, which is why a node with no
    // comment still puts "#name" on the air - "v#Anhaenger" and the like.
    //
    // The comment is filtered and capped the way upstream's
    // charset_filter does it for this field: 25 characters, and the six
    // bytes its own parsers read as delimiters are dropped. A space is
    // *not* one of them upstream, but decodeAPRSPOS() ends the field at
    // the first one, so a comment with a space arrives truncated - the
    // reason real stations here write "Meshcom-Region".
    char text[40] = "";
    size_t tp = 0;
    for (const char *p = cfg.meshComComment; *p && tp < sizeof(text) - 1; p++) {
        const char c = *p;
        if (c == '{' || c == '}' || c == ':' || c == ';' || c == ',' || c == '/') continue;
        if ((unsigned char)c < 0x20 || c == 0x7F) continue;
        text[tp++] = c;
    }
    text[tp] = '\0';

    // Set, the comment goes out exactly as written; empty, the position
    // still says which network it came out of, because "Meshcom" is what
    // this field falls back to.
    //
    // Both paths reach APRS-IS under the same callsign, so this text is
    // the only thing on the map that distinguishes them - but the operator
    // is the one who should word it. The APRS comment is not a candidate:
    // it is written for a different map, it can be 64 characters against
    // this field's 25, and a URL - the commonest APRS comment there is -
    // loses its ':' and '/' to the filter above and arrives as rubble.
    // Prefixing a configured comment was the other version of the same
    // mistake: it spends eight of the 25 bytes restating what the operator
    // already said in five ("vMesh").
    char info[64] = "";
    int used;
    if (tp) {
        used = snprintf(info, sizeof(info), "%s", text);
    } else {
        used = snprintf(info, sizeof(info), "Meshcom");
    }
    size_t ip = (used < 0) ? 0 : (size_t)used;
    if (ip > 25) ip = 25;               // upstream's cap on this field
    info[ip] = '\0';

    if (cfg.meshComName[0]) {
        snprintf(info + ip, sizeof(info) - ip, "#%s", cfg.meshComName);
    }

    // Battery ahead of altitude, in the order the rest of the mesh sends
    // them: ...symbol[text][/B=nnn]/A=nnnnnn. Omitted rather than faked
    // when the tracker has no voltage reading.
    char batt[8] = "";
    if (battPercent >= 0) {
        snprintf(batt, sizeof(batt), "/B=%03d", battPercent > 100 ? 100 : battPercent);
    }

    // Groups last, after every other tag, the order strconcat builds in
    // PositionToAPRS(). Upstream writes each group as "%i;", so the list
    // always ends in a semicolon; add one if the config left it off.
    char groups[40] = "";
    if (cfg.meshComGroups[0]) {
        const size_t gl = strlen(cfg.meshComGroups);
        snprintf(groups, sizeof(groups), "/R=%s%s", cfg.meshComGroups,
                 cfg.meshComGroups[gl - 1] == ';' ? "" : ";");
    }

    snprintf(out, outLen, "%07.2f%c%c%08.2f%c%c%s%s/A=%06d%s",
             latAprs, latHem, symTable, lonAprs, lonHem, symCode,
             info, batt, altFeet, groups);
    return true;
}

}  // namespace

// One MeshCom frame: data type identifier, message id, hop/mesh byte, the
// APRS-shaped text, then the trailer the decoder in the upstream tree reads
// back as [zero, hardware_id, lora_mod, fcs, fw, lasthw, fw_subver, ending]
// (extras/decode_meshcom.py). The FCS is a plain byte sum over everything
// ahead of it, high byte first - that decoder swaps the halves before
// comparing, which is the same thing said backwards.
static size_t meshComFrame(uint8_t *frame, size_t frameLen,
                           const TrackerConfig &cfg, uint8_t dti,
                           const char *aprs) {
    size_t n = 0;

    frame[n++] = dti;
    uint32_t msgId = meshComNextMsgId();
    frame[n++] = (uint8_t)(msgId & 0xFF);
    frame[n++] = (uint8_t)((msgId >> 8) & 0xFF);
    frame[n++] = (uint8_t)((msgId >> 16) & 0xFF);
    frame[n++] = (uint8_t)((msgId >> 24) & 0xFF);
    // Byte 5 carries the hop count in its low nibble and flags in the high
    // one: 0x10 mesh, 0x20 app-offline, 0x40 track (aprs_functions.cpp,
    // encodeAPRS). Only the mesh bit is set here, and track deliberately is
    // not - see the MeshCom section of CLAUDE.md for why claiming it costs
    // the station its mesh visibility.
    //
    // Two hop limits, split by data type the way initAPRS() splits them:
    // ':' and '@' take the text limit, a position the shorter one.
    const int hops = (dti == 0x3A || dti == 0x40) ? cfg.meshComHopText
                                                  : cfg.meshComMaxHop;
    frame[n++] = (uint8_t)((hops & 0x0F) | 0x10);

    size_t aprsLen = strlen(aprs);
    if (n + aprsLen + 10 >= frameLen) return 0;
    memcpy(frame + n, aprs, aprsLen);
    n += aprsLen;

    frame[n++] = 0x00;
    uint8_t hw = (uint8_t)cfg.meshComHardwareId;
    frame[n++] = hw;
    frame[n++] = meshComModulation(cfg);

    uint16_t fcs = 0;
    for (size_t i = 0; i < n; i++) fcs = (uint16_t)(fcs + frame[i]);
    frame[n++] = (uint8_t)((fcs >> 8) & 0xFF);
    frame[n++] = (uint8_t)(fcs & 0xFF);

    // The firmware-version byte is a protocol generation, not a product
    // label, and three receivers read it rather than print it:
    //
    //   aprs_functions.cpp:496  1..34 -> the whole frame is discarded,
    //                           "Packet discarded, wrong FW-version"
    //   lora_functions.cpp:763  >13   -> /A= is treated as feet and
    //   loop_functions.cpp:2992          converted to metres for display
    //
    // Sending this board's own 2.1 as "2" therefore put it in the discard
    // window of every current node, and left the altitude unconverted on
    // the ones old enough to accept it - 541 m arriving as "1775 m", which
    // is the sort of implausibility a map has every reason to drop. So the
    // value here is the generation the network requires to talk to us at
    // all; the sub-version byte after the FCS is where this firmware
    // identifies itself.
    frame[n++] = 35;
    frame[n++] = (uint8_t)(0x80 | hw);
    frame[n++] = 0x23;  // '#'
    frame[n++] = 0x7E;
    return n;
}

// Retune, key up, and come back to APRS. Leaving the radio on the MeshCom
// profile would silence the tracker's main product, so the way back runs
// even when the transmit itself failed.
static bool meshComTransmit(const TrackerConfig &cfg, const uint8_t *frame,
                            size_t n, int8_t drive) {
    char err[64];
    bool ok = false;
    if (TrackerRadio::setMode(RADIO_MODE_MESHCOM, cfg, err, sizeof(err))) {
        TrackerRadio::setDrive(drive);
        bool needPa = cfg.hasPa || TrackerRadio::hasModulePa();
        if (needPa) { digitalWrite(PIN_PA, HIGH); delay(250); watchdog_update(); }
        ok = TrackerRadio::send(frame, n);
        if (needPa) { delay(100); digitalWrite(PIN_PA, LOW); }
    }
    if (!TrackerRadio::setMode(RADIO_MODE_APRS, cfg, err, sizeof(err))) {
        Serial.printf("\x1b[1;5;31mRADIO STUCK OFF-PROFILE: %s\x1b[0m\r\n", err);
    }
    return ok;
}

void MeshCom::begin(const char *callsign, uint16_t seed) {
    uint32_t h = 2166136261UL;                  // FNV-1a over the callsign
    for (const char *p = callsign; p && *p; p++) {
        h ^= (uint8_t)*p;
        h *= 16777619UL;
    }
    meshComNodeId = h & 0x3FFFFFUL;
    meshComCounter = (uint16_t)(seed & 0x3FF);
}

bool MeshCom::sendPosition(const TrackerConfig &cfg, const char *callsign,
                           float lat, float lon, float altMeters, int8_t drive,
                           int battPercent) {
    if (!callsign || !callsign[0]) return false;

    char payload[128];
    if (!meshComPayload(payload, sizeof(payload), cfg, lat, lon, altMeters,
                        battPercent)) return false;

    char aprs[192];
    snprintf(aprs, sizeof(aprs), "%s>*!%s", callsign, payload);

    uint8_t frame[255];
    size_t n = meshComFrame(frame, sizeof(frame), cfg, 0x21, aprs);
    if (!n) return false;
    return meshComTransmit(cfg, frame, n, drive);
}

// The HEY probe: data type '@', destination "H", and a payload that starts
// as "R<neighbours>;". It is not a position and not a message - it asks the
// mesh who can hear this station, and every node that relays it appends its
// own signal report before passing it on, so the answer accumulates in the
// frame and reaches the server even though this board cannot receive a word
// of it. Read it back in a gateway's RX log.
//
// The neighbour count is the size of the sender's own MHeard list, and a
// transmit-only station has none, so it is honestly zero.
bool MeshCom::sendHey(const TrackerConfig &cfg, const char *callsign,
                      int8_t drive) {
    if (!callsign || !callsign[0]) return false;

    char aprs[64];
    snprintf(aprs, sizeof(aprs), "%s>H@R0;", callsign);

    uint8_t frame[255];
    size_t n = meshComFrame(frame, sizeof(frame), cfg, 0x40, aprs);
    if (!n) return false;
    return meshComTransmit(cfg, frame, n, drive);
}

// A private message. The wire shape is the same frame with ':' as the data
// type identifier and SOURCE>TARGET: ahead of the text - the upstream
// decoder splits on '>' and then on the DTI character itself, so the colon
// after the target is structural rather than decoration.
//
// The trailing {NNN is MeshCom's ack request. This tracker cannot receive,
// so no ack will ever arrive; it is sent anyway because a node that gets a
// message without one treats it as an ack-less broadcast and some clients
// then decline to show it in the message list.
bool MeshCom::sendMessage(const TrackerConfig &cfg, const char *callsign,
                          const char *target, const char *text, int8_t drive) {
    if (!callsign || !callsign[0]) return false;
    if (!target || !target[0]) return false;
    if (!text || !text[0]) return false;

    static uint16_t ackCounter = 1;

    char aprs[190];
    snprintf(aprs, sizeof(aprs), "%s>%s:%s{%03u", callsign, target, text,
             (unsigned)(ackCounter % 1000));
    ackCounter++;

    uint8_t frame[255];
    size_t n = meshComFrame(frame, sizeof(frame), cfg, 0x3A, aprs);
    if (!n) return false;
    return meshComTransmit(cfg, frame, n, drive);
}
