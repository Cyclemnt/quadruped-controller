#include "../include/leg.hpp"
#include <stdexcept>
#include <sstream>
#include <string>
#include <cmath>

Leg::Leg(LegID id_, PCA9685* driver, std::array<int, 3> channels)
    : id(id_), joints{{
            Joint(channels[0], driver),
            Joint(channels[1], driver),
            Joint(channels[2], driver)
        }} {}

Leg::~Leg() {
    for (int i = 0; i < joints.size(); i++)
        joints[i].~Joint();
}

// Set each joint to an angle
void Leg::setAngles(float q1, float q2, float q3) {
    checkAngleBounds(q1, q2, q3);
    
    joints[0].setAngle(M_PI_2 - q1 + offset1); // +30°, pi/2 - because upside down
    joints[1].setAngle(q2 + offset2); // +75°
    joints[2].setAngle(q3);
}

// Reset each joint
void Leg::reset() {
    setAngles(0.7854f, 0, 0); // 45, 0, 0°
}

// Check that joint angles are within the mechanical limits of the leg.
// Throws std::runtime_error if any angle is out of bounds.
void Leg::checkAngleBounds(float q1, float q2, float q3) const {
    if (q1 < -0.5236f || q1 > 2.0944f) {
        std::ostringstream oss;
        oss << "q1 out of bounds : " << q1 << " (ch " << joints[0].getChannel() << ")";
        throw std::runtime_error(oss.str());
    } else if (q2 < -1.309f || q2 > 1.309f) {
        std::ostringstream oss;
        oss << "q2 out of bounds : " << q2 << " (ch " << joints[1].getChannel() << ")";
        throw std::runtime_error(oss.str());
    } else if (q3 < 0 || q3 > 2.618f) {
        std::ostringstream oss;
        oss << "q3 out of bounds : " << q3 << " (ch " << joints[2].getChannel() << ")";
        throw std::runtime_error(oss.str());
    } else if (-q2 + q3 > 3.0f) {
        std::ostringstream oss;
        oss << "q3 - q2 out of bounds : " << -q2 + q3 << " (ch " << joints[1].getChannel() << " + " << joints[2].getChannel() << ")";
        throw std::runtime_error(oss.str());
    }
}

std::array<float, 3> Leg::getAngles() const {
    std::array<float, 3> angles;
    angles[0] = M_PI_2 - (joints[0].getAngle() - offset1); // -30°, pi/2 - because upside down
    angles[1] = joints[1].getAngle() - offset2; // -75°
    angles[2] = joints[2].getAngle();
    return angles;
}
