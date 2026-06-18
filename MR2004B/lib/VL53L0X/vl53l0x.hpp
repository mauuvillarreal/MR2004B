#pragma once

#include <stdint.h>
#include "hardware/i2c.h"

class VL53L0X {
public:
    explicit VL53L0X(i2c_inst_t* i2c, uint8_t address = 0x29);

    bool init(bool io_2v8 = true);
    bool testConnection();
    bool startContinuous();
    void stopContinuous();

    // Non-blocking: returns true only when a fresh valid sample was read.
    bool readDistanceMm(uint16_t& distance_mm);
    bool dataReady();

    uint16_t lastDistanceMm() const { return _last_distance_mm; }
    bool healthy() const { return _healthy; }

private:
    i2c_inst_t* _i2c;
    uint8_t _address;
    uint8_t _stop_variable;
    uint16_t _last_distance_mm;
    bool _healthy;
    bool _continuous;

    bool writeReg(uint8_t reg, uint8_t value);
    bool writeMulti(uint8_t reg, const uint8_t* data, uint8_t length);
    bool readReg(uint8_t reg, uint8_t& value);
    bool readMulti(uint8_t reg, uint8_t* data, uint8_t length);
    bool getSpadInfo(uint8_t& count, bool& aperture);
    bool performSingleRefCalibration(uint8_t vhv_init_byte);
};
