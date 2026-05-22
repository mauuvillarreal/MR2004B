#ifndef MPU6050_H
#define MPU6050_H

#include "hardware/i2c.h"

class MPU6050 {
public:
    MPU6050(i2c_inst_t* i2c, uint8_t addr = 0x68);

    bool init();
    bool testConnection();

    void readRaw(int16_t& ax, int16_t& ay, int16_t& az,
                 int16_t& gx, int16_t& gy, int16_t& gz);

    void read(float& ax, float& ay, float& az,
                    float& gx, float& gy, float& gz);

private:
    i2c_inst_t* _i2c;
    uint8_t _address;

    void writeReg(uint8_t reg, uint8_t data);
    void readReg(uint8_t reg, uint8_t* buffer, uint8_t length);
};

#endif
