#ifndef PCA9695_HPP
#define PCA9695_HPP

#include <cstdint>

class PCA9685 {
private:
    int fd;
    const uint8_t address = 0x40;
    const uint8_t MODE1 = 0x00;
    const uint8_t PRESCALE = 0xFE;

    void writeRegister(uint8_t reg, uint8_t value) const;

public:
    PCA9685();
    ~PCA9685();

    void setPWM(int channel, int on, int off) const;
    void disableAllPWM() const;
};

#endif // PCA9695_HPP
