#include "../include/robot.hpp"
#include "../include/kinematics.hpp"
#include "../include/constants.hpp"
#include <cmath>
#include <algorithm>
#include <chrono>
#include <thread>
#include <stdexcept>
#include <iostream>

Robot::Robot(PCA9685* driver, BNO055* imu_, Stabilizer* stabilizer_)
    : legs{{
            Leg(LegID::FL, driver, {0, 1, 2}),
            Leg(LegID::FR, driver, {3, 4, 5}),
            Leg(LegID::RR, driver, {6, 7, 8}),
            Leg(LegID::RL, driver, {9,10,11})
        }},
      imu(imu_), stabilizer(stabilizer_) {}

Robot::~Robot() {
    for (int i = 0; i < legs.size(); i++)
        legs[i].~Leg();
    if (imu != nullptr) imu->~BNO055();
    if (stabilizer != nullptr) stabilizer->~Stabilizer();
}

// Transform (x, y) to match the leg's coordinate system
void Robot::transformXY(LegID id, float& x, float& y) const {
    float newX, newY;
    switch (id) {
        case LegID::FL: newX =  x; newY =  y; break;
        case LegID::FR: newX = -y; newY =  x; break;
        case LegID::RR: newX = -x; newY = -y; break;
        case LegID::RL: newX =  y; newY = -x; break;
    }
    x = newX;
    y = newY;
}

// Move legs using basic linear interpolation
void Robot::moveLegs(
    std::vector<std::pair<LegID, std::array<float, 3>>> targetsLifted,
    std::vector<std::pair<LegID, std::array<float, 3>>> targetsFlat,
    bool transformTarget,
    float liftHeight,
    int steps
) {
    std::array<std::array<float, 3>, 4> pStart = getLegsPositions();

    auto processLeg = [&](LegID id, const std::array<float, 3>& target, bool lifted, float t) {
        int i = static_cast<int>(id);

        float x = (1 - t) * pStart[i][0] + t * target[0];
        float y = (1 - t) * pStart[i][1] + t * target[1];
        float z = (1 - t) * pStart[i][2] + t * target[2];

        if (lifted)
            z += liftHeight * (-std::cosh(M_PIf * (t - 0.5f)) + 2.5f) * 0.666667f;

        std::array<float, 3> q = Kinematics::computeIK(x, y, z);
        legs[i].setAngles(q[0], q[1], q[2]);
    };

    if (transformTarget) {
        for (auto& [id, pos] : targetsLifted)
            transformXY(id, pos[0], pos[1]);
        for (auto& [id, pos] : targetsFlat)
            transformXY(id, pos[0], pos[1]);
    }

    for (int step = 1; step <= steps; ++step) {
        float t = static_cast<float>(step) / steps;

        for (const auto& [id, target] : targetsLifted)
            processLeg(id, target, true, t);

        for (const auto& [id, target] : targetsFlat)
            processLeg(id, target, false, t);

        std::this_thread::sleep_for(std::chrono::milliseconds(STANDARD_DELAY));
    }
}

// Move legs using circular interpolation
void Robot::rotateLegs(std::vector<LegID>& legsLifted, std::vector<LegID>& legsFlat, float deltaAngleDeg, float liftHeight, int steps) {
    const float delta = deltaAngleDeg * M_PIf / 180.0f;

    std::array<std::array<float, 3>, 4> pStart = getLegsPositions();

    auto processLeg = [&](int i, bool lifted, float t) {
        float x0 = pStart[i][0];
        float y0 = pStart[i][1];
        float r = std::hypot(x0, y0);
        float theta0 = std::atan2(y0, x0);
        float theta = theta0 + (lifted ? +delta : -delta) * t;

        float x = r * std::cos(theta);
        float y = r * std::sin(theta);
        float z = STANDARD_HEIGHT;

        if (lifted)
            z += liftHeight * (-std::cosh(M_PIf * (t - 0.5f)) + 2.5f) * 0.666667f;

        std::array<float, 3> q = Kinematics::computeIK(x, y, z);
        legs[i].setAngles(q[0], q[1], q[2]);
    };

    for (int step = 1; step <= steps; ++step) {
        float t = static_cast<float>(step) / steps;

        for (LegID id : legsLifted)
            processLeg(static_cast<int>(id), true, t);

        for (LegID id : legsFlat)
            processLeg(static_cast<int>(id), false, t);

        std::this_thread::sleep_for(std::chrono::milliseconds(STANDARD_DELAY));
    }
}

// Reset all legs to 0°, 0°, 0°
void Robot::resetLegs() {
    for (int i = 0; i < 4; i++)
        legs[i].reset();
}

// Standard stance
void Robot::rest() {
    std::array<float, 3> p = {RESTING_X, RESTING_Y, RESTING_H};
    std::vector<std::pair<LegID, std::array<float, 3>>> targets = {{LegID::FL, p}, {LegID::FR, p}, {LegID::RR, p}, {LegID::RL, p}};
    moveLegs({}, targets, false, 0, 1);
}

// Sit (change height)
void Robot::sit(bool down) {
    float zTarget = down ? SITTING_LOWER_HEIGHT : SITTING_HIGHER_HEIGHT;

    std::array<std::array<float, 3>, 4> pStart = getLegsPositions();
    std::array<std::array<float, 3>, 4> pTargets;

    for (int i = 0; i < 4; ++i) {
        pTargets[i] = pStart[i];  // Target = Start
        pTargets[i][2] = zTarget; // Except zTarget
    }

    std::vector<std::pair<LegID, std::array<float, 3>>> targets = {{LegID::FL, pTargets[0]}, {LegID::FR, pTargets[1]}, {LegID::RR, pTargets[2]}, {LegID::RL, pTargets[3]}};
    moveLegs({}, targets, false);
}

// Walk forward
void Robot::walk() {
    float h = WALKING_BODY_HEIGHT;
    float dx = WALKING_LEG_DISTANCE_FROM_BODY;  // Distance from body to end of leg on the sides
    float dy = WALKING_LEG_DISTANCE_ALLOWED_BETWEEN_FRONT_AND_BACK;  // Distance the leg gets under body front/back
    float stride = 2 * dx + 2 * dy; // Distance a step adds
    float shift = stride - dx - dy; // Distance a shift covers

    // Initial position
    std::vector<std::pair<LegID, std::array<float, 3>>> targets = {
        {LegID::FL, { dx,  dx, h}}, // FL
        {LegID::FR, {-dy, -dx, h}}, // FR
        {LegID::RR, { dy, -dx, h}}, // RR
        {LegID::RL, {-dx,  dx, h}}  // RL
    };
    moveLegs({}, targets);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Gait algorithm
    for (int i = 0; i < 2; ++i) {
        moveLegs({{LegID::FR, {stride - dy, -dx, h}}}, {}); // STEP 1
        targets = {
            {LegID::FL, {    -dy,  dx, h}}, // FL   x = dx - shift
            {LegID::FR, {     dx, -dx, h}}, // FR   x = stride - dy - shift
            {LegID::RR, {    -dx, -dx, h}}, // RR   x = dy - shift
            {LegID::RL, {-stride,  dx, h}}  // RL   x = -dx - shift
        };
        moveLegs({}, targets);                              // SHIFT 1
        moveLegs({{LegID::RL, {dy, dx, h}}}, {});           // STEP 2   x = -stride + stride + dy
        moveLegs({{LegID::FL, {stride, dx, h}}}, {});       // STEP 3   x = -dy + dy + stride
        targets = {
            {LegID::FL, {     dx,  dx, h}}, // FL   x = stride - shift
            {LegID::FR, {    -dy, -dx, h}}, // FR   x = dx - shift
            {LegID::RR, {-stride, -dx, h}}, // RR   x = -dx - shift
            {LegID::RL, {    -dx,  dx, h}}  // RL   x = dy - shift
        };
        moveLegs({}, targets);                              // SHIFT 2
        moveLegs({{LegID::RR, {dy, -dx, h}}}, {});          // STEP 4   x = -stride + stride
    }
}

void Robot::run() {
    constexpr float h = RUNNING_BODY_HEIGHT;
    constexpr float dx = RUNNING_LEG_DISTANCE_FROM_BODY;   // Distance from body to end of leg on the sides
    constexpr float step = RUNNING_STEP_SIZE;  // Distance a steps adds
    constexpr float stepHeight = RUNNING_STEP_HEIGHT;
    constexpr int pointsPerMovement = RUNNING_POINTS_PER_MOVEMENT;

    // Initial position
    std::vector<std::pair<LegID, std::array<float, 3>>> targetFlats = {
        {LegID::FL, {dx, dx, h}}, // FL
        {LegID::FR, {dx, dx, h}}, // FR
        {LegID::RR, {dx, dx, h}}, // RR
        {LegID::RL, {dx, dx, h}}  // RL
    };
    moveLegs({}, targetFlats, 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(2000));

    // Gait algorithm
    for (int i = 0; i < 20; ++i) {
        moveLegs({{LegID::FL, {dx + step, dx, h}}, {LegID::RR, {dx - step, dx, h}}},
                 {{LegID::FR, {dx, dx - step, h}}, {LegID::RL, {dx, dx + step, h}}}, 0, stepHeight, pointsPerMovement);
//        moveLegs({{LegID::FR, {dx, dx, h}}, {LegID::RL, {dx, dx, h}}},
//                 {{LegID::FL, {dx, dx, h}}, {LegID::RR, {dx, dx, h}}}, 0, stepHeight, pointsPerMovement);
        moveLegs({{LegID::FR, {dx, dx + step, h}}, {LegID::RL, {dx, dx - step, h}}},
                 {{LegID::FL, {dx - step, dx, h}}, {LegID::RR, {dx + step, dx, h}}}, 0, stepHeight, pointsPerMovement);
//        moveLegs({{LegID::FL, {dx, dx, h}}, {LegID::RR, {dx, dx, h}}},
//                 {{LegID::FR, {dx, dx, h}}, {LegID::RL, {dx, dx, h}}}, 0, stepHeight, pointsPerMovement);
    }
    
    moveLegs({{LegID::FL, {dx, dx, h}}, {LegID::RR, {dx, dx, h}}},
             {{LegID::FR, {dx, dx, h}}, {LegID::RL, {dx, dx, h}}}, 0, stepHeight, pointsPerMovement);
}

// Turn left
void Robot::turn() {
    constexpr float angle = TURNING_ANGLE_A_STEP_COVERS;

    rest();
    std::vector<LegID> a = {LegID::FL, LegID::RR};
    std::vector<LegID> b = {LegID::FR, LegID::RL};
    for (int i = 0; i < 5; ++i) {
        rotateLegs(a, b, angle); // FL and RR lifted and going clockwise
        rotateLegs(b, a, angle); // FR and RL lifted and going clockwise
    }
}

float Robot::normalizeAngle(float angle) {
    while (angle > 180.0f) angle -= 360.0f;
    while (angle <= -180.0f) angle += 360.0f;
    return angle;
}

// Try to stay at level
void Robot::level() {
    if (!imu) throw std::runtime_error("IMU not initialized");
    if (!stabilizer) throw std::runtime_error("Stabilizer not initialized");

    auto euler = imu->getEuler();
    auto positions = getLegsPositions();

    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - lastUpdate).count();
    if (dt <= 0) dt = 0.01f;
    lastUpdate = now;

    stabilizer->computeOffsets(euler[2], euler[1], dt, positions);

    // Conversion direct -> moveLegs
    std::vector<std::pair<LegID, std::array<float,3>>> targets;
    for (int i = 0; i < 4; ++i) {
        targets.push_back({static_cast<LegID>(i), positions[i]});
    }

    moveLegs({}, targets, false, 0, 1);
}

std::array<float, 3> Robot::getLegPosition(LegID id) const {
    std::array<float, 3> p;
    std::array<float, 3> q = legs[static_cast<int>(id)].getAngles();
    p = Kinematics::computeFK(q[0], q[1], q[2]);
    return p;
}

std::array<std::array<float, 3>, 4> Robot::getLegsPositions() const {
    std::array<std::array<float, 3>, 4> ps;
    for (int i = 0; i < 4; ++i) {
        std::array<float, 3> q = legs[i].getAngles();
        std::array<float, 3> p = Kinematics::computeFK(q[0], q[1], q[2]);
        ps[i] = p;
    }
    return ps;
}
