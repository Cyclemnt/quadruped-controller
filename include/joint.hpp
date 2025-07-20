#ifndef JOINT_HPP
#define JOINT_HPP

#include "pca9685.hpp"

class Joint {
private:
    int channel;
    PCA9685* driver;

    float currentAngle;

    const float servoRange = 2.618f; //(155 - 5)°
    const int pulseMin = 102;  // 500 µs
    const int pulseMax = 512;  // 2500 µs

public:
    Joint(int channel_, PCA9685* driver_);
    ~Joint();

    int angleToPWM(float angle) const;
    void setAngle(float angle);

    float getAngle() const;
    float getChannel() const;
};

#endif // JOINT_HPP
