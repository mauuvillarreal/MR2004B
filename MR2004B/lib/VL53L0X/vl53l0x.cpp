#include "vl53l0x.hpp"

#include "pico/stdlib.h"

namespace {
constexpr uint8_t REG_SYSRANGE_START = 0x00;
constexpr uint8_t REG_SYSTEM_SEQUENCE_CONFIG = 0x01;
constexpr uint8_t REG_SYSTEM_INTERRUPT_CONFIG_GPIO = 0x0A;
constexpr uint8_t REG_SYSTEM_INTERRUPT_CLEAR = 0x0B;
constexpr uint8_t REG_RESULT_INTERRUPT_STATUS = 0x13;
constexpr uint8_t REG_RESULT_RANGE_STATUS = 0x14;
constexpr uint8_t REG_GPIO_HV_MUX_ACTIVE_HIGH = 0x84;
constexpr uint8_t REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV = 0x89;
constexpr uint8_t REG_IDENTIFICATION_MODEL_ID = 0xC0;
constexpr uint8_t REG_MSRC_CONFIG_CONTROL = 0x60;
constexpr uint8_t REG_FINAL_RANGE_MIN_COUNT_RATE = 0x44;
constexpr uint8_t REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0 = 0xB0;
constexpr uint8_t REG_DYNAMIC_SPAD_REF_EN_START_OFFSET = 0x4F;
constexpr uint8_t REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD = 0x4E;
constexpr uint8_t REG_GLOBAL_CONFIG_REF_EN_START_SELECT = 0xB6;
constexpr uint32_t INIT_TIMEOUT_MS = 200;
}

VL53L0X::VL53L0X(i2c_inst_t* i2c, uint8_t address)
    : _i2c(i2c),
      _address(address),
      _stop_variable(0),
      _last_distance_mm(0),
      _healthy(false),
      _continuous(false) {}

bool VL53L0X::writeReg(uint8_t reg, uint8_t value) {
    uint8_t data[2] = {reg, value};
    return i2c_write_blocking(_i2c, _address, data, 2, false) == 2;
}

bool VL53L0X::writeMulti(uint8_t reg, const uint8_t* data, uint8_t length) {
    if (length > 15) return false;
    uint8_t buffer[16];
    buffer[0] = reg;
    for (uint8_t i = 0; i < length; ++i) buffer[i + 1] = data[i];
    return i2c_write_blocking(_i2c, _address, buffer, length + 1, false) == (length + 1);
}

bool VL53L0X::readReg(uint8_t reg, uint8_t& value) {
    if (i2c_write_blocking(_i2c, _address, &reg, 1, true) != 1) return false;
    return i2c_read_blocking(_i2c, _address, &value, 1, false) == 1;
}

bool VL53L0X::readMulti(uint8_t reg, uint8_t* data, uint8_t length) {
    if (i2c_write_blocking(_i2c, _address, &reg, 1, true) != 1) return false;
    return i2c_read_blocking(_i2c, _address, data, length, false) == length;
}

bool VL53L0X::testConnection() {
    uint8_t model = 0;
    return readReg(REG_IDENTIFICATION_MODEL_ID, model) && model == 0xEE;
}

bool VL53L0X::getSpadInfo(uint8_t& count, bool& aperture) {
    uint8_t value = 0;
    if (!writeReg(0x80, 0x01) || !writeReg(0xFF, 0x01) || !writeReg(0x00, 0x00)) return false;
    if (!writeReg(0xFF, 0x06) || !readReg(0x83, value) || !writeReg(0x83, value | 0x04)) return false;
    if (!writeReg(0xFF, 0x07) || !writeReg(0x81, 0x01) || !writeReg(0x80, 0x01) ||
        !writeReg(0x94, 0x6B) || !writeReg(0x83, 0x00)) return false;

    uint32_t start = to_ms_since_boot(get_absolute_time());
    do {
        if (!readReg(0x83, value)) return false;
        if (value != 0) break;
        if ((to_ms_since_boot(get_absolute_time()) - start) > INIT_TIMEOUT_MS) return false;
        sleep_us(100);
    } while (true);

    if (!writeReg(0x83, 0x01) || !readReg(0x92, value)) return false;
    count = value & 0x7F;
    aperture = ((value >> 7) & 0x01) != 0;

    if (!writeReg(0x81, 0x00) || !writeReg(0xFF, 0x06) || !readReg(0x83, value) ||
        !writeReg(0x83, value & ~0x04) || !writeReg(0xFF, 0x01) ||
        !writeReg(0x00, 0x01) || !writeReg(0xFF, 0x00) || !writeReg(0x80, 0x00)) return false;
    return true;
}

bool VL53L0X::performSingleRefCalibration(uint8_t vhv_init_byte) {
    if (!writeReg(REG_SYSRANGE_START, 0x01 | vhv_init_byte)) return false;
    uint32_t start = to_ms_since_boot(get_absolute_time());
    uint8_t status = 0;
    do {
        if (!readReg(REG_RESULT_INTERRUPT_STATUS, status)) return false;
        if ((status & 0x07) != 0) break;
        if ((to_ms_since_boot(get_absolute_time()) - start) > INIT_TIMEOUT_MS) return false;
        sleep_us(100);
    } while (true);
    return writeReg(REG_SYSTEM_INTERRUPT_CLEAR, 0x01) && writeReg(REG_SYSRANGE_START, 0x00);
}

bool VL53L0X::init(bool io_2v8) {
    _healthy = false;
    _continuous = false;
    if (!testConnection()) return false;

    uint8_t value = 0;
    if (io_2v8) {
        if (!readReg(REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, value) ||
            !writeReg(REG_VHV_CONFIG_PAD_SCL_SDA_EXTSUP_HV, value | 0x01)) return false;
    }

    if (!writeReg(0x88, 0x00) || !writeReg(0x80, 0x01) || !writeReg(0xFF, 0x01) ||
        !writeReg(0x00, 0x00) || !readReg(0x91, _stop_variable) || !writeReg(0x00, 0x01) ||
        !writeReg(0xFF, 0x00) || !writeReg(0x80, 0x00)) return false;

    if (!readReg(REG_MSRC_CONFIG_CONTROL, value) || !writeReg(REG_MSRC_CONFIG_CONTROL, value | 0x12)) return false;
    const uint8_t signal_rate_limit[2] = {0x00, 0x20}; // 0.25 MCPS, Q9.7
    if (!writeMulti(REG_FINAL_RANGE_MIN_COUNT_RATE, signal_rate_limit, 2) ||
        !writeReg(REG_SYSTEM_SEQUENCE_CONFIG, 0xFF)) return false;

    uint8_t spad_count = 0;
    bool spad_aperture = false;
    if (!getSpadInfo(spad_count, spad_aperture)) return false;

    uint8_t spad_map[6] = {};
    if (!readMulti(REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, spad_map, 6)) return false;
    if (!writeReg(0xFF, 0x01) || !writeReg(REG_DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00) ||
        !writeReg(REG_DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C) || !writeReg(0xFF, 0x00) ||
        !writeReg(REG_GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4)) return false;

    uint8_t first_spad = spad_aperture ? 12 : 0;
    uint8_t enabled = 0;
    for (uint8_t i = 0; i < 48; ++i) {
        if (i < first_spad || enabled == spad_count) spad_map[i / 8] &= ~(1u << (i % 8));
        else if ((spad_map[i / 8] >> (i % 8)) & 1u) ++enabled;
    }
    if (!writeMulti(REG_GLOBAL_CONFIG_SPAD_ENABLES_REF_0, spad_map, 6)) return false;

    // ST default tuning sequence.
    const uint8_t tuning[][2] = {
        {0xFF,0x01},{0x00,0x00},{0xFF,0x00},{0x09,0x00},{0x10,0x00},{0x11,0x00},
        {0x24,0x01},{0x25,0xFF},{0x75,0x00},{0xFF,0x01},{0x4E,0x2C},{0x48,0x00},
        {0x30,0x20},{0xFF,0x00},{0x30,0x09},{0x54,0x00},{0x31,0x04},{0x32,0x03},
        {0x40,0x83},{0x46,0x25},{0x60,0x00},{0x27,0x00},{0x50,0x06},{0x51,0x00},
        {0x52,0x96},{0x56,0x08},{0x57,0x30},{0x61,0x00},{0x62,0x00},{0x64,0x00},
        {0x65,0x00},{0x66,0xA0},{0xFF,0x01},{0x22,0x32},{0x47,0x14},{0x49,0xFF},
        {0x4A,0x00},{0xFF,0x00},{0x7A,0x0A},{0x7B,0x00},{0x78,0x21},{0xFF,0x01},
        {0x23,0x34},{0x42,0x00},{0x44,0xFF},{0x45,0x26},{0x46,0x05},{0x40,0x40},
        {0x0E,0x06},{0x20,0x1A},{0x43,0x40},{0xFF,0x00},{0x34,0x03},{0x35,0x44},
        {0xFF,0x01},{0x31,0x04},{0x4B,0x09},{0x4C,0x05},{0x4D,0x04},{0xFF,0x00},
        {0x44,0x00},{0x45,0x20},{0x47,0x08},{0x48,0x28},{0x67,0x00},{0x70,0x04},
        {0x71,0x01},{0x72,0xFE},{0x76,0x00},{0x77,0x00},{0xFF,0x01},{0x0D,0x01},
        {0xFF,0x00},{0x80,0x01},{0x01,0xF8},{0xFF,0x01},{0x8E,0x01},{0x00,0x01},
        {0xFF,0x00},{0x80,0x00}
    };
    for (const auto& pair : tuning) if (!writeReg(pair[0], pair[1])) return false;

    if (!writeReg(REG_SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04) ||
        !readReg(REG_GPIO_HV_MUX_ACTIVE_HIGH, value) ||
        !writeReg(REG_GPIO_HV_MUX_ACTIVE_HIGH, value & ~0x10) ||
        !writeReg(REG_SYSTEM_INTERRUPT_CLEAR, 0x01) ||
        !writeReg(REG_SYSTEM_SEQUENCE_CONFIG, 0xE8)) return false;

    if (!writeReg(REG_SYSTEM_SEQUENCE_CONFIG, 0x01) || !performSingleRefCalibration(0x40) ||
        !writeReg(REG_SYSTEM_SEQUENCE_CONFIG, 0x02) || !performSingleRefCalibration(0x00) ||
        !writeReg(REG_SYSTEM_SEQUENCE_CONFIG, 0xE8)) return false;

    _healthy = true;
    return true;
}

bool VL53L0X::startContinuous() {
    if (!_healthy) return false;
    if (!writeReg(0x80, 0x01) || !writeReg(0xFF, 0x01) || !writeReg(0x00, 0x00) ||
        !writeReg(0x91, _stop_variable) || !writeReg(0x00, 0x01) ||
        !writeReg(0xFF, 0x00) || !writeReg(0x80, 0x00) ||
        !writeReg(REG_SYSRANGE_START, 0x02)) return false;
    _continuous = true;
    return true;
}

void VL53L0X::stopContinuous() {
    if (!_continuous) return;
    writeReg(REG_SYSRANGE_START, 0x01);
    writeReg(0xFF, 0x01);
    writeReg(0x00, 0x00);
    writeReg(0x91, 0x00);
    writeReg(0x00, 0x01);
    writeReg(0xFF, 0x00);
    _continuous = false;
}

bool VL53L0X::dataReady() {
    if (!_healthy || !_continuous) return false;
    uint8_t status = 0;
    if (!readReg(REG_RESULT_INTERRUPT_STATUS, status)) {
        _healthy = false;
        return false;
    }
    return (status & 0x07) != 0;
}

bool VL53L0X::readDistanceMm(uint16_t& distance_mm) {
    if (!dataReady()) return false;
    uint8_t data[2] = {};
    if (!readMulti(REG_RESULT_RANGE_STATUS + 10, data, 2) ||
        !writeReg(REG_SYSTEM_INTERRUPT_CLEAR, 0x01)) {
        _healthy = false;
        return false;
    }
    distance_mm = static_cast<uint16_t>((data[0] << 8) | data[1]);
    _last_distance_mm = distance_mm;
    return true;
}
