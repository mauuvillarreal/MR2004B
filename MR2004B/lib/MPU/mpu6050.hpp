#ifndef MPU6050_H
#define MPU6050_H

#include "hardware/i2c.h"
#include "pico/stdlib.h"
#include <stdint.h>

class MPU6050 {
public:
    enum class AccelRange : uint8_t {
        G2  = 0,
        G4  = 1,
        G8  = 2,
        G16 = 3,
    };

    enum class GyroRange : uint8_t {
        DPS250  = 0,
        DPS500  = 1,
        DPS1000 = 2,
        DPS2000 = 3,
    };

    MPU6050(i2c_inst_t* i2c, uint8_t addr = 0x68);

    bool init(AccelRange accel_range = AccelRange::G8,
              GyroRange gyro_range = GyroRange::DPS500,
              uint8_t dlpf_cfg = 3,
              uint8_t sample_divider = 4);
    bool testConnection();

    bool configure(AccelRange accel_range,
                   GyroRange gyro_range,
                   uint8_t dlpf_cfg,
                   uint8_t sample_divider);

    bool readRaw(int16_t& ax, int16_t& ay, int16_t& az,
                 int16_t& gx, int16_t& gy, int16_t& gz);

    bool read(float& ax, float& ay, float& az,
              float& gx, float& gy, float& gz);

    float accelScaleLsbPerG() const { return _accel_lsb_per_g; }
    float gyroScaleLsbPerDps() const { return _gyro_lsb_per_dps; }

private:
    i2c_inst_t* _i2c;
    uint8_t _address;
    float _accel_lsb_per_g;
    float _gyro_lsb_per_dps;

    bool writeReg(uint8_t reg, uint8_t data);
    bool readReg(uint8_t reg, uint8_t* buffer, uint8_t length);
    void updateScales(AccelRange accel_range, GyroRange gyro_range);
};

#endif
