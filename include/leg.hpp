#ifndef LEG_HPP
#define LEG_HPP

#include "joint.hpp"
#include <array>

enum class LegID {FL = 0, FR = 1, RR = 2, RL = 3};

class Leg {
private:
    LegID id;
    std::array<Joint, 3> joints;
    const float offset1 = 0.5236f; // 30°
    const float offset2 = 1.309f;  // 75°

public:
    Leg(LegID id_, PCA9685* driver, std::array<int, 3> channels);
    ~Leg();

    void setAngles(float q1, float q2, float q3);
    void reset();

    void checkAngleBounds(float q1, float q2, float q3) const;

    std::array<float, 3> getAngles() const;
};

#endif // LEG_HPP
