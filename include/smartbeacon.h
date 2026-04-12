// ============================================================
// RF.Guru LoRa 433 APRS Tracker - SmartBeacon Algorithm
// Intelligent beaconing with corner pegging and jitter lock
// ============================================================

#pragma once

#include <Arduino.h>
#include <math.h>

class SmartBeacon {
public:
    void reset() {
        _lastBeaconTime = millis() - 9999000UL;
        _lastLat = 999.0f;
        _lastLon = 999.0f;
        _lastHeading = -1.0f;
        _lastHeadingTime = 0;
        _filteredHeading = -1.0f;
    }

    static float angleDiff(float a, float b) {
        float diff = fabsf(a - b);
        if (diff > 180.0f) diff = 360.0f - diff;
        return diff;
    }

    float filterHeading(float heading, bool useFilter) {
        if (heading < 0) return -1.0f;
        if (!useFilter) return heading;

        if (_filteredHeading < 0) {
            _filteredHeading = heading;
            return heading;
        }

        // Wrap-safe low-pass filter
        float delta = fmodf(heading - _filteredHeading + 540.0f, 360.0f) - 180.0f;
        _filteredHeading = fmodf(_filteredHeading + 0.3f * delta + 360.0f, 360.0f);
        return _filteredHeading;
    }

    void headingChange(float heading, unsigned long nowMs, float &diff, float &slope) {
        diff = 0;
        slope = 0;

        if (heading < 0) return;

        if (_lastHeading < 0 || _lastHeadingTime == 0) {
            _lastHeading = heading;
            _lastHeadingTime = nowMs;
            return;
        }

        diff = angleDiff(heading, _lastHeading);
        float dt = (nowMs - _lastHeadingTime) / 1000.0f;
        slope = diff / max(dt, 0.1f);

        _lastHeading = heading;
        _lastHeadingTime = nowMs;
    }

    float intervalForSpeed(float speedKmh, int slowSpeed, int fastSpeed,
                           int slowRate, int fastRate) {
        if (speedKmh <= (float)slowSpeed) return (float)slowRate;
        if (speedKmh >= (float)fastSpeed) return (float)fastRate;

        float frac = (speedKmh - slowSpeed) / (float)(fastSpeed - slowSpeed);
        return slowRate - frac * (slowRate - fastRate);
    }

    bool shouldBeacon(float lat, float lon, float speedKmh, float heading,
                      float stationarySpeed, int slowSpeed, int fastSpeed,
                      int slowRate, int fastRate, int turnThreshold,
                      float turnSlope, bool headingFilterEnabled) {
        unsigned long now = millis();
        float dtSec = (now - _lastBeaconTime) / 1000.0f;

        if (_lastLat > 90.0f) {
            float fh = filterHeading(heading, headingFilterEnabled);
            float d, s;
            headingChange(fh, now, d, s);
            return true;
        }

        if (speedKmh < 0) speedKmh = 0;

        float fh = filterHeading(heading, headingFilterEnabled);
        float diff, slope;
        headingChange(fh, now, diff, slope);

        // Stationary -> slowRate
        if (speedKmh < stationarySpeed) {
            return dtSec >= (float)slowRate;
        }

        float interval = intervalForSpeed(speedKmh, slowSpeed, fastSpeed,
                                          slowRate, fastRate);
        if (dtSec >= interval) return true;

        // Corner pegging (rate limited by fastRate)
        if (fh >= 0 && dtSec >= (float)fastRate) {
            if (diff >= (float)turnThreshold) return true;
            if (slope >= turnSlope) return true;
        }

        return false;
    }

    void updateAfterBeacon(float lat, float lon) {
        _lastBeaconTime = millis();
        _lastLat = lat;
        _lastLon = lon;
    }

private:
    unsigned long _lastBeaconTime = 0;
    float _lastLat = 999.0f;
    float _lastLon = 999.0f;
    float _lastHeading = -1.0f;
    unsigned long _lastHeadingTime = 0;
    float _filteredHeading = -1.0f;
};

// Stationary jitter lock
class JitterLock {
public:
    void reset() {
        _anchorLat = 999.0f;
        _anchorLon = 999.0f;
        _stationary = false;
        _exitHits = 0;
    }

    // Returns true if position was stabilized (using anchor),
    // sets startedMoving if transitioning from stationary to moving
    bool stabilize(float &lat, float &lon, float speedKmh,
                   float stationarySpeed, int stationaryDistance,
                   int stationaryExitCount, bool &startedMoving,
                   float &movedM) {
        startedMoving = false;
        movedM = 0;

        if (lat > 90.0f || lon > 180.0f) return false;
        if (speedKmh < 0) speedKmh = 0;

        if (_anchorLat > 90.0f) {
            _anchorLat = lat;
            _anchorLon = lon;
            _stationary = (speedKmh < stationarySpeed);
            _exitHits = 0;
            return false;
        }

        movedM = haversineM(_anchorLat, _anchorLon, lat, lon);

        if (_stationary) {
            if (movedM >= (float)stationaryDistance) {
                _exitHits++;
                if (_exitHits >= stationaryExitCount) {
                    _stationary = false;
                    _exitHits = 0;
                    _anchorLat = lat;
                    _anchorLon = lon;
                    startedMoving = true;
                    return false;
                }
                lat = _anchorLat;
                lon = _anchorLon;
                return true;
            }
            _exitHits = 0;
            lat = _anchorLat;
            lon = _anchorLon;
            return true;
        }

        // Not stationary - follow GPS
        _anchorLat = lat;
        _anchorLon = lon;
        _exitHits = 0;
        if (speedKmh < stationarySpeed) {
            _stationary = true;
        }
        return false;
    }

    bool isStationary() const { return _stationary; }

private:
    static float haversineM(float lat1, float lon1, float lat2, float lon2) {
        float dlat = radians(lat2 - lat1);
        float dlon = radians(lon2 - lon1);
        float a = sinf(dlat / 2) * sinf(dlat / 2) +
                  cosf(radians(lat1)) * cosf(radians(lat2)) *
                  sinf(dlon / 2) * sinf(dlon / 2);
        float c = 2 * atan2f(sqrtf(a), sqrtf(1 - a));
        return 6371000.0f * c;
    }

    float _anchorLat = 999.0f;
    float _anchorLon = 999.0f;
    bool _stationary = false;
    int _exitHits = 0;
};
