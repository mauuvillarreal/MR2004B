#include "mpu6050.hpp"

// Reegisters
#define PWR_MGMT_1     0x6B
#define WHO_AM_I       0x75
#define ACCEL_XOUT_H   0x3B


// Constructor
MPU6050::MPU6050(i2c_inst_t* i2c, uint8_t addr)
    : _i2c(i2c), _address(addr) {}


// Initialize MPU6050
bool MPU6050::init() {
    writeReg(PWR_MGMT_1, 0x00); // Wake up
    sleep_ms(100);
    return testConnection();
}

// Verify device ID
bool MPU6050::testConnection() {
    uint8_t id;
    readReg(WHO_AM_I, &id, 1);
    return (id == 0x68);
}

void MPU6050::writeReg(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    i2c_write_blocking(_i2c, _address, buf, 2, false);
}

void MPU6050::readReg(uint8_t reg, uint8_t* buffer, uint8_t length) {
    i2c_write_blocking(_i2c, _address, &reg, 1, true);
    i2c_read_blocking(_i2c, _address, buffer, length, false);
}

void MPU6050::readRaw(int16_t& ax, int16_t& ay, int16_t& az,
                      int16_t& gx, int16_t& gy, int16_t& gz) {

    uint8_t data[14];
    readReg(ACCEL_XOUT_H, data, 14);

    ax = (data[0] << 8) | data[1];
    ay = (data[2] << 8) | data[3];
    az = (data[4] << 8) | data[5];

    gx = (data[8] << 8) | data[9];
    gy = (data[10] << 8) | data[11];
    gz = (data[12] << 8) | data[13];
}

void MPU6050::read(float& ax, float& ay, float& az,
                         float& gx, float& gy, float& gz) {

    int16_t rax, ray, raz, rgx, rgy, rgz;
    readRaw(rax, ray, raz, rgx, rgy, rgz);

    // Default sensitivity:
    // Accel ±2g → 16384 LSB/g
    // Gyro ±250°/s → 131 LSB/(°/s)

    ax = rax / 16384.0f;
    ay = ray / 16384.0f;
    az = raz / 16384.0f;

    gx = rgx / 131.0f;
    gy = rgy / 131.0f;
    gz = rgz / 131.0f;
}
