#include "../include/bno055.hpp"
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdexcept>
#include <cstring>
#include <array>

#define BNO055_OPR_MODE      0x3D
#define BNO055_PAGE_ID       0x07
#define BNO055_PWR_MODE      0x3E
#define BNO055_SYS_TRIGGER   0x3F
#define BNO055_EULER_H_LSB   0x1A
#define BNO055_ACCEL_DATA_X_LSB 0x08
#define BNO055_GYRO_DATA_X_LSB  0x14

BNO055::BNO055() {
    fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0) throw std::runtime_error("Failed to open I2C bus");

    if (ioctl(fd, I2C_SLAVE, address) < 0) {
        close(fd);
        throw std::runtime_error("Failed to set I2C address");
    }

    initialize();
}

BNO055::~BNO055() {
    close(fd);
}

// Write value into register
void BNO055::writeRegister(uint8_t reg, uint8_t value) const {
    uint8_t buffer[2] = {reg, value};
    if (write(fd, buffer, 2) != 2) {
        throw std::runtime_error("I2C: Failed to write");
    }
}

bool BNO055::readLen(uint8_t reg, uint8_t* buffer, uint8_t len) const {
    if (write(fd, &reg, 1) != 1) return false;
    return (read(fd, buffer, len) == len);
}

void BNO055::initialize() const {
    writeRegister(BNO055_OPR_MODE, 0x00);  // Mode config
    usleep(20000);
    writeRegister(BNO055_PWR_MODE, 0x00);  // Normal power mode
    usleep(10000);
    writeRegister(BNO055_PAGE_ID, 0x00);   // Page 0
    writeRegister(BNO055_OPR_MODE, 0x0C);  // NDOF fusion mode
    usleep(20000);
}

std::array<float, 3> BNO055::getEuler() const {
    uint8_t buffer[6];
    if (!readLen(BNO055_EULER_H_LSB, buffer, 6))
        throw std::runtime_error("Failed to read Euler angles");

    int16_t heading = (buffer[1] << 8) | buffer[0];
    int16_t roll    = (buffer[3] << 8) | buffer[2];
    int16_t pitch   = (buffer[5] << 8) | buffer[4];

    return { heading / 16.0f, roll / 16.0f, pitch / 16.0f };
}

std::array<float, 3> BNO055::getAccel() const {
    uint8_t buffer[6];
    if (!readLen(BNO055_ACCEL_DATA_X_LSB, buffer, 6))
        throw std::runtime_error("Failed to read Accel");

    int16_t x = (buffer[1] << 8) | buffer[0];
    int16_t y = (buffer[3] << 8) | buffer[2];
    int16_t z = (buffer[5] << 8) | buffer[4];

    return { x / 100.0f, y / 100.0f, z / 100.0f };
}

std::array<float, 3> BNO055::getGyro() const {
    uint8_t buffer[6];
    if (!readLen(BNO055_GYRO_DATA_X_LSB, buffer, 6))
        throw std::runtime_error("Failed to read Gyro");

    int16_t x = (buffer[1] << 8) | buffer[0];
    int16_t y = (buffer[3] << 8) | buffer[2];
    int16_t z = (buffer[5] << 8) | buffer[4];

    return { x / 16.0f, y / 16.0f, z / 16.0f };
}
