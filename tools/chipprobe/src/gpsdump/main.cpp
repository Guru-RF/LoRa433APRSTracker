// ============================================================
// RF.Guru GPS survey
//
// Echoes the raw GPS UART to USB serial and tallies which NMEA talker
// IDs and sentence types actually arrive. V1 boards carry a u-blox NEO;
// V2 boards carry a Zhongke ATGM336H (AT6558), which speaks NMEA plus
// the CASIC/PCAS command set rather than UBX - so the talker IDs it
// emits decide whether TinyGPSPlus can parse it at all.
//
// TinyGPSPlus accepts $GP, $GN, $GA, $GB and $GL. It does NOT accept
// the older $BD BeiDou talker, so this survey is how we find out.
//
// Radio untouched, PA held low, nothing transmitted.
// ============================================================

#include <Arduino.h>
#include <SerialPIO.h>

#include "pins.h"

static SerialPIO gpsSerial(PIN_GPS_TX, PIN_GPS_RX, 256);

// Compose an NMEA sentence and append the checksum, so command strings
// stay readable in the source.
static void gpsSendNmea(const char *body) {
    uint8_t cs = 0;
    for (const char *p = body; *p; p++) cs ^= (uint8_t)*p;
    gpsSerial.printf("$%s*%02X\r\n", body, cs);
    Serial.printf("  -> $%s*%02X\r\n", body, cs);
}

struct Talker {
    char id[4];
    uint32_t count;
};
static Talker talkers[12];
static size_t talkerCount = 0;

static void noteSentence(const char *s) {
    if (s[0] != '$') return;
    char id[4] = { s[1], s[2], 0, 0 };
    for (size_t i = 0; i < talkerCount; i++) {
        if (strcmp(talkers[i].id, id) == 0) { talkers[i].count++; return; }
    }
    if (talkerCount < sizeof(talkers) / sizeof(talkers[0])) {
        strncpy(talkers[talkerCount].id, id, 3);
        talkers[talkerCount].count = 1;
        talkerCount++;
    }
}

void setup() {
    pinMode(PIN_PA, OUTPUT);        digitalWrite(PIN_PA, LOW);
    pinMode(PIN_I2C_PWR, OUTPUT);   digitalWrite(PIN_I2C_PWR, LOW);
    pinMode(PIN_LED_PWR, OUTPUT);   digitalWrite(PIN_LED_PWR, HIGH);
    pinMode(PIN_LORA_CS, OUTPUT);   digitalWrite(PIN_LORA_CS, HIGH);

    // Same reset the tracker does: hold low 1 s, release, wait 1 s.
    pinMode(PIN_GPS_RST, OUTPUT);
    digitalWrite(PIN_GPS_RST, LOW);
    delay(1000);
    digitalWrite(PIN_GPS_RST, HIGH);
    delay(1000);

    Serial.begin(115200);
    delay(3000);

    Serial.print("\r\n=== GPS survey ===\r\n");

    // The module's TX line idles high whether or not it is talking, so
    // silence at 9600 could equally be a different baud rate. Sweep the
    // plausible ones and look at the raw bytes: framing errors from a
    // wrong rate still produce bytes, a dead module produces none.
    static const uint32_t BAUDS[] = { 9600, 115200, 38400, 57600, 19200, 4800 };
    uint32_t best = 0, bestPrintable = 0;

    for (size_t i = 0; i < sizeof(BAUDS) / sizeof(BAUDS[0]); i++) {
        gpsSerial.end();
        delay(50);
        gpsSerial.begin(BAUDS[i]);
        delay(50);
        while (gpsSerial.available()) gpsSerial.read();   // flush

        uint32_t total = 0, printable = 0, dollars = 0;
        uint8_t first[16];
        size_t firstLen = 0;
        uint32_t t0 = millis();
        while (millis() - t0 < 2500) {
            while (gpsSerial.available()) {
                uint8_t b = gpsSerial.read();
                total++;
                if (b == '$') dollars++;
                if ((b >= 0x20 && b < 0x7F) || b == '\r' || b == '\n') printable++;
                if (firstLen < sizeof(first)) first[firstLen++] = b;
            }
        }

        Serial.printf("  %6lu baud: %4lu bytes, %lu printable, %lu '$'",
                      (unsigned long)BAUDS[i], (unsigned long)total,
                      (unsigned long)printable, (unsigned long)dollars);
        if (firstLen) {
            Serial.print("  first:");
            for (size_t j = 0; j < firstLen; j++) Serial.printf(" %02X", first[j]);
        }
        Serial.print("\r\n");

        if (total > 8 && printable > bestPrintable) {
            bestPrintable = printable;
            best = BAUDS[i];
        }
    }

    if (best) Serial.printf("=> best match: %lu baud\r\n", (unsigned long)best);
    else      Serial.println("=> GPS is SILENT at every rate tried");

    gpsSerial.end();
    delay(50);
    gpsSerial.begin(best ? best : 9600);
    Serial.printf("listening at %lu baud, raw NMEA follows\r\n",
                  (unsigned long)(best ? best : 9600));
}

void loop() {
    static char line[128];
    static size_t len = 0;
    static uint32_t lastReport = 0;
    static uint32_t sentences = 0;
    static bool sentPcas = false;

    while (gpsSerial.available()) {
        char c = gpsSerial.read();
        if (c == '\r') continue;
        if (c == '\n') {
            line[len] = '\0';
            if (len > 0) {
                Serial.printf("%s\r\n", line);
                noteSentence(line);
                sentences++;
            }
            len = 0;
        } else if (len < sizeof(line) - 1) {
            line[len++] = c;
        } else {
            len = 0;   // overlong, resync
        }
    }

    uint32_t now = millis();

    // After 15 s of listening, try the CASIC/PCAS configuration the
    // ATGM336H understands and show whether the output changes.
    if (!sentPcas && now > 15000) {
        sentPcas = true;
        Serial.println("\r\n--- sending PCAS config (ATGM336H/AT6558) ---");
        gpsSendNmea("PCAS02,1000");                 // 1 Hz fix rate
        delay(200);
        gpsSendNmea("PCAS03,1,0,0,0,1,0,0,0");      // GGA + RMC only
        delay(200);
        gpsSendNmea("PCAS04,3");                    // GPS + BeiDou
        delay(200);
        Serial.println("--- continuing ---\r\n");
    }

    if (now - lastReport > 10000) {
        lastReport = now;
        Serial.printf("\r\n[%lus] %lu sentences; talkers:",
                      (unsigned long)(now / 1000), (unsigned long)sentences);
        for (size_t i = 0; i < talkerCount; i++) {
            Serial.printf(" $%s=%lu", talkers[i].id, (unsigned long)talkers[i].count);
        }
        if (talkerCount == 0) Serial.print(" (none - no data from the GPS)");
        Serial.print("\r\n\r\n");
    }
}
