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
    for (int i = 0; i < 4; ++i) {
        float x = legPositions[i][0]; // in mm
        float y = legPositions[i][1]; // in mm

        // Option A: exact geometry
        float dz = - x * std::tan(corrPitch) + y * std::tan(corrRoll);

        // Option B: small-angle linearization (faster, ok for small angles)
        // float dz = - x * corrPitch + y * corrRoll;

        // Optionally apply an overall scaling / safety limit
        const float scale = 1.0f; // tune this if necessary
        dz *= scale;

        // apply and clamp
        legPositions[i][2] += dz;
        legPositions[i][2] = std::clamp(legPositions[i][2], -250.0f, 250.0f);
    }
}

float Stabilizer::normalizeAngle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle <= -180.0f) angle += 360.0f;
    return angle;
}