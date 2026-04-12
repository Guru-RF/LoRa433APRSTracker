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
static int aprsPosition(char *buf, float lat, float lon,
                          float speed_kmh, float course, const char *symbol) {
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
