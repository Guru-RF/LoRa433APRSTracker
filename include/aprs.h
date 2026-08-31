// ============================================================
// RF.Guru LoRa 433 APRS Tracker - APRS Frame Encoding
// Compressed position format with telemetry
// ============================================================

#pragma once

#include <Arduino.h>
#include <math.h>

// Make APRS timestamp: DDHHMMz or HHMMSSh
static void aprsTimestamp(char *buf, char format, int day, int hour, int minute, int second) {
    if (format == 'z') {
        snprintf(buf, 8, "%02d%02d%02dz", day, hour, minute);
    } else {
        snprintf(buf, 8, "%02d%02d%02dh", hour, minute, second);
    }
}

// Make compressed APRS position string
// Returns length written to buf (13 chars + null)
// symbol: 2 chars, e.g. "L>" (table + code)
// speed_kmh: negative = no speed/course
// course: 0-360 degrees, negative = no course
// alt_ft < 0 means "no altitude". It is only used when there is no course
// or speed to send: the two are mutually exclusive in the cs field, and for
// a tracker course/speed is the more useful of the two while moving.
static int aprsPosition(char *buf, float lat, float lon,
                          float speed_kmh, float course, const char *symbol,
                          float alt_ft = -1.0f) {
    if (lat < -89.99999f || lat > 89.99999f ||
        lon < -179.99999f || lon > 179.99999f) {
        buf[0] = '\0';
        return 0;
    }

    char symTable = '/';
    char symCode = '/';
    if (strlen(symbol) >= 2) {
        symTable = symbol[0];
        symCode = symbol[1];
    }

    // Convert digit overlay to lowercase letter
    if (symTable >= '0' && symTable <= '9') {
        symTable = symTable - '0' + 'a';
    }

    // Base91-encode latitude
    double latVal = 380926.0 * (90.0 - lat);
    char latStr[5];
    for (int i = 3; i >= 0; i--) {
        double divisor = pow(91.0, i);
        int v = (int)(latVal / divisor);
        latVal = fmod(latVal, divisor);
        latStr[3 - i] = (char)(v + 33);
    }
    latStr[4] = '\0';

    // Base91-encode longitude
    double lonVal = 190463.0 * (180.0 + lon);
    char lonStr[5];
    for (int i = 3; i >= 0; i--) {
        double divisor = pow(91.0, i);
        int v = (int)(lonVal / divisor);
        lonVal = fmod(lonVal, divisor);
        lonStr[3 - i] = (char)(v + 33);
    }
    lonStr[4] = '\0';

    int pos = 0;
    buf[pos++] = symTable;
    memcpy(buf + pos, latStr, 4); pos += 4;
    memcpy(buf + pos, lonStr, 4); pos += 4;
    buf[pos++] = symCode;

    // Speed and course
    if (speed_kmh >= 0 && course > 0 && course <= 360) {
        int cval = (int)((course + 2) / 4);
        if (cval > 89) cval = 0;
        buf[pos++] = (char)(cval + 33);

        float knots = speed_kmh / 1.852f;
        int speedNum = (int)(log(knots + 1) / log(1.08f) + 0.5f);
        if (speedNum > 89) speedNum = 89;
        buf[pos++] = (char)(speedNum + 33);
        buf[pos++] = 'A';
    } else if (alt_ft > 0.0f) {
        // > 0 rather than > 1: a coastal tracker sitting at 0.0-0.3 m would
        // otherwise get neither the cs bytes nor a /A=, losing altitude
        // entirely in the one band where it is most likely to be. The
        // clamp below maps it to cs=0, and this still dodges logf(0).

        // Altitude in the cs bytes - APRS 1.0.1 chapter 9, page 40: with the
        // T byte's NMEA-source bits set to GGA (bits 4,3 = 10), cs carries
        // altitude = 1.002^cs feet.
        //
        // This is free. The same page says that when c is a space, meaning no
        // course, speed or range, "the csT bytes are ignored" - so a
        // stationary report was spending three bytes on nothing while
        // altitude went out separately as a nine-byte /A= in the comment.
        int cs = (int)(logf(alt_ft) / logf(1.002f) + 0.5f);
        if (cs < 0) cs = 0;
        if (cs > 91 * 91 - 1) cs = 91 * 91 - 1;
        buf[pos++] = (char)(cs / 91 + 33);
        buf[pos++] = (char)(cs % 91 + 33);
        // GPS fix current (bit 5) | NMEA source GGA (bits 4,3 = 10) = 48.
        buf[pos++] = (char)(48 + 33);
    } else {
        buf[pos++] = ' ';
        buf[pos++] = ' ';
        buf[pos++] = 'A';
    }

    buf[pos] = '\0';
    return pos;
}

// Base91 encode a non-negative integer (min 2 chars)
static int base91Encode(char *buf, int n) {
    if (n < 0) n = 0;
    char tmp[8];
    int len = 0;
    do {
        tmp[len++] = (char)(33 + (n % 91));
        n /= 91;
    } while (n > 0 && len < 7);

    // Pad to min 2 chars
    if (len == 1) {
        tmp[len++] = '!';
    }

    // Reverse into buf
    for (int i = 0; i < len; i++) {
        buf[i] = tmp[len - 1 - i];
    }
    buf[len] = '\0';
    return len;
}
