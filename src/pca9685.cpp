#include "../include/pca9685.hpp"
#include <iostream>
#include <unistd.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <stdexcept>

PCA9685::PCA9685() {
    fd = open("/dev/i2c-1", O_RDWR);
    if (fd < 0)
        throw std::runtime_error("Failed to open I2C bus");

    if (ioctl(fd, I2C_SLAVE, address) < 0) {
        close(fd);
        throw std::runtime_error("Failed to set I2C address");
    }

    initialize();
}

PCA9685::~PCA9685() {
    disableAllPWM();
    close(fd);
}

// Write value into register
void PCA9685::writeRegister(uint8_t reg, uint8_t value) const {
    uint8_t buffer[2] = {reg, value};
    if (write(fd, buffer, 2) != 2) {
        throw std::runtime_error("I2C: Failed to write");
    }
}

// Initialize the chip
void PCA9685::initialize() const {
    writeRegister(MODE1, 0x10);       // Sleep
    writeRegister(PRESCALE, 121);     // 50 Hz
    writeRegister(MODE1, 0x00);
    usleep(500);
    writeRegister(MODE1, 0xA1);       // Auto-increment
    
    disableAllPWM();
}

// Send PWM signal to specific channel
void PCA9685::setPWM(int channel, int on, int off) const {
    uint8_t reg = 0x06 + 4 * channel;
    uint8_t data[5] = {
        reg,
        static_cast<uint8_t>(on & 0xFF),
        static_cast<uint8_t>(on >> 8),
        static_cast<uint8_t>(off & 0xFF),
        static_cast<uint8_t>(off >> 8)
    };
    if (write(fd, data, 5) != 5) {
        throw std::runtime_error("Erreur envoi PWM");
    }
}

// Stop sending signals to all channels
void PCA9685::disableAllPWM() const {
    for (int ch = 0; ch < 16; ++ch)
        setPWM(ch, 0, 0);
}
