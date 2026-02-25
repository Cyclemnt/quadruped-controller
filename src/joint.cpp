#include "../include/joint.hpp"

Joint::Joint(int channel_, PCA9685* driver_)
    : channel(channel_), driver(driver_), currentAngle(0.0f) {}

Joint::~Joint() {}

// Convert angle (rad) to PWM signal
int Joint::angleToPWM(float angle) const {
    return pulseMin + angle * (pulseMax - pulseMin) / static_cast<float>(servoRange);
}

// Set joint to angle
void Joint::setAngle(float angle) {
    if (angle < 0) angle = 0;
    if (angle > servoRange) angle = servoRange;
    int pwm = angleToPWM(angle);
    driver->setPWM(channel, 0, pwm);
    currentAngle = angle;
}

float Joint::getAngle() const {
    return currentAngle;
}

float Joint::getChannel() const {
    return channel;
}
