#ifndef STABILIZER_HPP
#define STABILIZER_HPP

#include "pid.hpp"
#include <array>

class Stabilizer {
private:
    PID pidRoll;
    PID pidPitch;
    float normalizeAngle(float angle);

public:
    Stabilizer(float kp = 2.0f, float ki = 0.0f, float kd = 0.5f);
    ~Stabilizer();

    // Returns z offsets for {FL, FR, RR, RL}
    void computeOffsets(float roll_deg, float pitch_deg, float dt, std::array<std::array<float, 3>, 4>& legPositions);
};

#endif // STABILIZER_HPP
