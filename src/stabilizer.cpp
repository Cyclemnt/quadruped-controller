#include "../include/stabilizer.hpp"
#include <cmath>
#include <algorithm>

Stabilizer::Stabilizer(float kp, float ki, float kd)
    : pidRoll(kp, ki, kd), pidPitch(kp, ki, kd) {}

Stabilizer::~Stabilizer() { pidRoll.~PID(); pidPitch.~PID(); }

// Returns z offsets for {FL, FR, RR, RL}
void Stabilizer::computeOffsets(
    float roll_deg, float pitch_deg, float dt,
    std::array<std::array<float, 3>, 4>& legPositions
) {
    float roll = normalizeAngle(roll_deg - 180);
    float pitch = normalizeAngle(pitch_deg);

    // Convert to radians
    const float roll_rad = roll * (M_PI / 180.0f);
    const float pitch_rad = pitch * (M_PI / 180.0f);

    // PID outputs: you can choose PID to output a scale factor (unitless) or directly angular correction.
    // Here we use PID to compute a scalar "intensity" for roll and pitch (in radians) :
    float corrRoll = pidRoll.update(-roll_deg, dt);
    float corrPitch = pidPitch.update(-pitch_deg, dt);

    // For each leg compute geometric dz
    for (int i = 0; i < 4; i++) {
        // Fetch positions from center of body to end of legs
        float x = legPositions[i][0] + 70;
        float y = legPositions[i][1] + 70;
        // Compute offset
        float dz = ((i < 2) ? 1 : -1) * x * std::tan(pitch_rad) + ((i == 0 || i == 3) ? 1 : -1) * y * std::tan(roll_rad);
        // Apply offset
        legPositions[i][2] += dz;
        legPositions[i][2] = std::clamp(legPositions[i][2], -220.0f, -80.0f);
    }
}

float Stabilizer::normalizeAngle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle <= -180.0f) angle += 360.0f;
    return angle;
}