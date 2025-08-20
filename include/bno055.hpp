#ifndef BNOO55_HPP
#define BNOO55_HPP

#include <array>
#include <cstdint>

class BNO055 {
private:
    int fd; // File descriptor
    const uint8_t address = 0x28; // I²C address

    bool write8(uint8_t reg, uint8_t value);
    bool readLen(uint8_t reg, uint8_t* buffer, uint8_t len);

public:
    BNO055();
    ~BNO055();

    bool begin();
    std::array<float, 3> getEuler();   // [heading, roll, pitch] °
    std::array<float, 3> getAccel();   // Acceleration m/s²
    std::array<float, 3> getGyro();    // Angular speed °/s
};

#endif // BNOO55_HPP