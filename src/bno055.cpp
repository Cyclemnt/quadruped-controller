#include "../include/bno055.hpp"
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <iostream>

#define BNO055_OPR_MODE      0x3D
#define BNO055_SYS_TRIGGER   0x3F
#define BNO055_UNIT_SEL      0x3B
#define BNO055_CALIB_STAT    0x35
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
void BNO055::write8(uint8_t reg, uint8_t value) const {
    uint8_t buffer[2] = {reg, value};
    if (write(fd, buffer, 2) != 2)
        throw std::runtime_error("I2C: Failed to write");
}

uint8_t BNO055::read8(uint8_t reg) const {
    uint8_t value;
    if (!readLen(reg, &value, 1))
        throw std::runtime_error("I2C: Failed to read8");
    return value;
}

bool BNO055::readLen(uint8_t reg, uint8_t* buffer, uint8_t len) const {
    struct i2c_rdwr_ioctl_data packets;
    struct i2c_msg messages[2];

    // Premier message : on écrit l'adresse du registre
    messages[0].addr  = address;
    messages[0].flags = 0; // write
    messages[0].len   = 1;
    messages[0].buf   = &reg;

    // Deuxième message : on lit 'len' octets
    messages[1].addr  = address;
    messages[1].flags = I2C_M_RD;
    messages[1].len   = len;
    messages[1].buf   = buffer;

    packets.msgs  = messages;
    packets.nmsgs = 2;

    if (ioctl(fd, I2C_RDWR, &packets) < 0) {
        perror("I2C_RDWR");
        return false;
    }
    return true;
}

void BNO055::initialize() const {
    // Config mode
    write8(BNO055_OPR_MODE, 0x00);
    usleep(20000);

    // Verify chip ID
    uint8_t id = read8(0x00);
    if (id != 0xA0)
        throw std::runtime_error("BNO055 not detected (wrong chip ID)");

    // Sensor reset
    write8(BNO055_SYS_TRIGGER, 0x20);
    usleep(650000); // 650ms pour redémarrer

    // Wait for ID availability
    while (read8(0x00) != 0xA0)
        usleep(10000);

    // Degrees unit
    write8(BNO055_UNIT_SEL, 0x00);

    // Activate external crystal
    write8(BNO055_SYS_TRIGGER, 0x80);
    usleep(10000);

    // NDOF mode
    write8(BNO055_OPR_MODE, 0x0C);
    usleep(20000);

    // System status verification
    uint8_t sysStatus = read8(BNO055_CALIB_STAT);
    if (sysStatus != 0x05)
        std::cerr << "Warning: BNO055 system not fully initialized (status=" 
                  << static_cast<int>(sysStatus) << ")" << std::endl;

    std::cout << "BNO055 initialized in NDOF mode with external crystal" << std::endl;
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
