#ifndef BNOO55_HPP
#define BNOO55_HPP

#include <array>
#include <cstdint>

class BNO055 {
private:
    int fd; // File descriptor
    const uint8_t address = 0x28; // I²C address

    void write8(uint8_t reg, uint8_t value) const;
    uint8_t read8(uint8_t reg) const;
    bool readLen(uint8_t reg, uint8_t* buffer, uint8_t len) const;

public:
    BNO055();
    ~BNO055();

    void initialize() const;

    std::array<float, 3> getEuler() const;   // [heading, roll, pitch] °
    std::array<float, 3> getAccel() const;   // Acceleration m/s²
    std::array<float, 3> getGyro() const;    // Angular speed °/s
};

#endif // BNOO55_HPP