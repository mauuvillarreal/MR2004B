#include "mpu6050.hpp"

#define MPU6050_SMPLRT_DIV     0x19
#define MPU6050_CONFIG         0x1A
#define MPU6050_GYRO_CONFIG    0x1B
#define MPU6050_ACCEL_CONFIG   0x1C
#define MPU6050_PWR_MGMT_1     0x6B
#define MPU6050_WHO_AM_I       0x75
#define MPU6050_ACCEL_XOUT_H   0x3B

MPU6050::MPU6050(i2c_inst_t* i2c, uint8_t addr)
    : _i2c(i2c),
      _address(addr),
      _accel_lsb_per_g(4096.0f),   // default after init(): ±8 g
      _gyro_lsb_per_dps(65.5f) {}  // default after init(): ±500 dps

bool MPU6050::init(AccelRange accel_range,
                   GyroRange gyro_range,
                   uint8_t dlpf_cfg,
                   uint8_t sample_divider) {
    // Wake up and use the X gyro PLL as clock source. This is more stable than
    // the internal oscillator once the device is running.
    if (!writeReg(MPU6050_PWR_MGMT_1, 0x01)) {
        return false;
    }
    sleep_ms(100);

    if (!testConnection()) {
        return false;
    }

    return configure(accel_range, gyro_range, dlpf_cfg, sample_divider);
}

bool MPU6050::testConnection() {
    uint8_t id = 0;
    if (!readReg(MPU6050_WHO_AM_I, &id, 1)) {
        return false;
    }
    return (id == 0x68);
}

bool MPU6050::configure(AccelRange accel_range,
                        GyroRange gyro_range,
                        uint8_t dlpf_cfg,
                        uint8_t sample_divider) {
    if (dlpf_cfg > 6) {
        dlpf_cfg = 3;
    }

    // DLPF=3 gives a useful vibration-filtered signal for a moving carriage.
    // sample_divider=4 gives 200 Hz sample rate when DLPF is enabled.
    if (!writeReg(MPU6050_CONFIG, dlpf_cfg)) return false;
    if (!writeReg(MPU6050_SMPLRT_DIV, sample_divider)) return false;

    uint8_t accel_cfg = (static_cast<uint8_t>(accel_range) & 0x03u) << 3;
    uint8_t gyro_cfg  = (static_cast<uint8_t>(gyro_range)  & 0x03u) << 3;

    if (!writeReg(MPU6050_ACCEL_CONFIG, accel_cfg)) return false;
    if (!writeReg(MPU6050_GYRO_CONFIG, gyro_cfg)) return false;

    updateScales(accel_range, gyro_range);
    sleep_ms(20);
    return true;
}

bool MPU6050::writeReg(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    return i2c_write_blocking(_i2c, _address, buf, 2, false) == 2;
}

bool MPU6050::readReg(uint8_t reg, uint8_t* buffer, uint8_t length) {
    if (i2c_write_blocking(_i2c, _address, &reg, 1, true) != 1) {
        return false;
    }
    return i2c_read_blocking(_i2c, _address, buffer, length, false) == length;
}

void MPU6050::updateScales(AccelRange accel_range, GyroRange gyro_range) {
    switch (accel_range) {
        case AccelRange::G2:  _accel_lsb_per_g = 16384.0f; break;
        case AccelRange::G4:  _accel_lsb_per_g = 8192.0f;  break;
        case AccelRange::G8:  _accel_lsb_per_g = 4096.0f;  break;
        case AccelRange::G16: _accel_lsb_per_g = 2048.0f;  break;
    }

    switch (gyro_range) {
        case GyroRange::DPS250:  _gyro_lsb_per_dps = 131.0f; break;
        case GyroRange::DPS500:  _gyro_lsb_per_dps = 65.5f;  break;
        case GyroRange::DPS1000: _gyro_lsb_per_dps = 32.8f;  break;
        case GyroRange::DPS2000: _gyro_lsb_per_dps = 16.4f;  break;
    }
}

bool MPU6050::readRaw(int16_t& ax, int16_t& ay, int16_t& az,
                      int16_t& gx, int16_t& gy, int16_t& gz) {
    uint8_t data[14];
    if (!readReg(MPU6050_ACCEL_XOUT_H, data, 14)) {
        ax = ay = az = gx = gy = gz = 0;
        return false;
    }

    ax = (int16_t)((data[0] << 8) | data[1]);
    ay = (int16_t)((data[2] << 8) | data[3]);
    az = (int16_t)((data[4] << 8) | data[5]);

    gx = (int16_t)((data[8]  << 8) | data[9]);
    gy = (int16_t)((data[10] << 8) | data[11]);
    gz = (int16_t)((data[12] << 8) | data[13]);
    return true;
}

bool MPU6050::read(float& ax, float& ay, float& az,
                   float& gx, float& gy, float& gz) {
    int16_t rax, ray, raz, rgx, rgy, rgz;
    if (!readRaw(rax, ray, raz, rgx, rgy, rgz)) {
        ax = ay = az = gx = gy = gz = 0.0f;
        return false;
    }

    ax = rax / _accel_lsb_per_g;
    ay = ray / _accel_lsb_per_g;
    az = raz / _accel_lsb_per_g;

    gx = rgx / _gyro_lsb_per_dps;
    gy = rgy / _gyro_lsb_per_dps;
    gz = rgz / _gyro_lsb_per_dps;
    return true;
}
