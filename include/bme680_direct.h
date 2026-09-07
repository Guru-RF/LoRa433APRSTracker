#pragma once

// The BME680 driven through Bosch's bme68x API directly, with a rail
// power-cycle standing in for the soft reset.
//
// Why not Adafruit_BME680, which is what brings the Bosch code in: its
// begin() calls bme68x_init(), which opens with a soft reset (0xE0 = 0xB6)
// unconditionally, and one of these sensors ACKs that write and then NAKs
// everything afterwards - with the bus idle high, no stuck line, after
// 610 ms, after a 9-clock recovery, after a master re-init. Nothing short
// of cutting its power brings it back. gas_sensor is private in that class,
// so there is no way to skip the reset from outside it. Whether that part
// is defective or counterfeit is not something firmware can settle, but
// the driver should not be what stops a healthy one from working.
//
// So the bus callbacks are ours, the write callback swallows that one
// command, and begin() resets the part by dropping its rail instead -
// with SDA and SCL held low meanwhile. Left high, the pull-ups feed the
// part through its ESD diodes and it never powers down at all.
//
// Settings are Adafruit_BME680::begin(initSettings=true) verbatim, gas
// heater included, so readings and the bme680TempOffset users calibrated
// against them stay comparable. Init here reads the chip ID, variant and
// the whole calibration block, so a begin() that returns true means the
// part is genuinely talking; performReading() records which register and
// which Wire code failed when it does not, because a car has no console.

#include <Wire.h>
#include <Adafruit_BME680.h>   // brings in bme68x.h and the BME68X_* constants
#include "hardware/watchdog.h"

// Cut the sensor rail properly. Both lines low first, or the part is
// back-powered through its protection diodes and sits half-alive.
static inline void i2cRailOff(TwoWire &wire, int pwrPin, int sdaPin, int sclPin) {
    wire.end();
    pinMode(sdaPin, OUTPUT);
    pinMode(sclPin, OUTPUT);
    digitalWrite(sdaPin, LOW);
    digitalWrite(sclPin, LOW);
    digitalWrite(pwrPin, LOW);
}

class Bme680Direct {
public:
    float temperature = 0.0f;   // deg C
    float humidity    = 0.0f;   // %RH
    float pressure    = 0.0f;   // Pa

    // Diagnostics from the last failure: which step, which register, and
    // the Wire code (write) or byte count (100 + n, read).
    int8_t  lastModeRslt = 0;
    int8_t  lastDataRslt = 0;
    uint8_t lastFields   = 0;
    uint8_t errReg()  const { return _ctx.errReg; }
    int     errCode() const { return _ctx.errCode; }
    bool    errRead() const { return _ctx.errRead; }

    bool begin(TwoWire *wire, uint8_t addr, int pwrPin, int sdaPin, int sclPin,
               uint32_t clkHz) {
        _ctx.wire = wire;
        _ctx.addr = addr;

        i2cRailOff(*wire, pwrPin, sdaPin, sclPin);
        delay(300);
        watchdog_update();
        pinMode(sdaPin, INPUT);
        pinMode(sclPin, INPUT);
        digitalWrite(pwrPin, HIGH);
        delay(500);
        watchdog_update();

        wire->setSDA(sdaPin);
        wire->setSCL(sclPin);
        wire->setClock(clkHz);
        wire->begin();

        memset(&_dev, 0, sizeof(_dev));
        _dev.intf     = BME68X_I2C_INTF;
        _dev.intf_ptr = &_ctx;
        _dev.read     = &i2cRead;
        _dev.write    = &i2cWrite;
        _dev.delay_us = &delayUsec;
        _dev.amb_temp = 25;

        if (bme68x_init(&_dev) != BME68X_OK) return false;

        _conf.filter  = BME68X_FILTER_SIZE_3;
        _conf.odr     = BME68X_ODR_NONE;
        _conf.os_hum  = BME68X_OS_2X;
        _conf.os_pres = BME68X_OS_4X;
        _conf.os_temp = BME68X_OS_8X;
        if (bme68x_set_conf(&_conf, &_dev) != BME68X_OK) return false;

        _heat.enable     = BME68X_ENABLE;
        _heat.heatr_temp = 320;
        _heat.heatr_dur  = 150;
        if (bme68x_set_heatr_conf(BME68X_FORCED_MODE, &_heat, &_dev) != BME68X_OK)
            return false;

        // Left asleep. Forced mode is one-shot and the first reading
        // triggers its own; Adafruit starts one here for no benefit.
        return true;
    }

    bool performReading() {
        lastModeRslt = bme68x_set_op_mode(BME68X_FORCED_MODE, &_dev);
        if (lastModeRslt != BME68X_OK) return false;

        uint32_t us = bme68x_get_meas_dur(BME68X_FORCED_MODE, &_conf, &_dev) +
                      (uint32_t)_heat.heatr_dur * 1000;

        // Twice the computed duration, as Adafruit waits, and no bus
        // traffic meanwhile. About 370 ms with the heater on.
        delay(us / 1000 * 2 + 2);
        watchdog_update();

        struct bme68x_data data;
        for (int tries = 0; tries < 5; tries++) {
            lastFields = 0;
            lastDataRslt = bme68x_get_data(BME68X_FORCED_MODE, &data, &lastFields, &_dev);
            if (lastDataRslt == BME68X_OK && lastFields > 0) {
                temperature = data.temperature;
                humidity    = data.humidity;
                pressure    = data.pressure;
                return true;
            }
            if (lastDataRslt != BME68X_W_NO_NEW_DATA) break;   // a bus error, not "not yet"
            delay(50);
            watchdog_update();
        }
        return false;
    }

private:
    struct Ctx {
        TwoWire *wire    = nullptr;
        uint8_t  addr    = 0;
        uint8_t  errReg  = 0;
        int      errCode = 0;
        bool     errRead = false;
    };

    static BME68X_INTF_RET_TYPE i2cRead(uint8_t reg, uint8_t *data, uint32_t len,
                                        void *intfPtr) {
        Ctx *c = (Ctx *)intfPtr;
        c->wire->beginTransmission(c->addr);
        c->wire->write(reg);
        int e = c->wire->endTransmission(false);
        if (e != 0) { c->errReg = reg; c->errCode = e; c->errRead = true; return -1; }
        size_t got = c->wire->requestFrom(c->addr, (size_t)len);
        if (got != len) { c->errReg = reg; c->errCode = 100 + (int)got; c->errRead = true; return -1; }
        for (uint32_t i = 0; i < len; i++) data[i] = c->wire->read();
        return 0;
    }

    static BME68X_INTF_RET_TYPE i2cWrite(uint8_t reg, const uint8_t *data,
                                         uint32_t len, void *intfPtr) {
        // The one write this sensor may not come back from. begin() has
        // already reset the part by cutting its power, so report success.
        if (reg == BME68X_REG_SOFT_RESET && len == 1 && data[0] == BME68X_SOFT_RESET_CMD)
            return 0;

        Ctx *c = (Ctx *)intfPtr;
        c->wire->beginTransmission(c->addr);
        c->wire->write(reg);
        for (uint32_t i = 0; i < len; i++) c->wire->write(data[i]);
        int e = c->wire->endTransmission();
        if (e != 0) { c->errReg = reg; c->errCode = e; c->errRead = false; }
        return e == 0 ? 0 : -1;
    }

    static void delayUsec(uint32_t us, void *) {
        delay(us / 1000);
        yield();
        delayMicroseconds(us % 1000);
    }

    struct bme68x_dev        _dev;
    struct bme68x_conf       _conf;
    struct bme68x_heatr_conf _heat;
    Ctx                      _ctx;
};
