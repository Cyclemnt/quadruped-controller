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
      imu(imu_), stabilizer(stabilizer_), bodyHeight(RESTING_H), runningStepSize(RUNNING_STEP_SIZE), turningStepAngle(TURNING_ANGLE_A_STEP_COVERS) {}

Robot::~Robot() {
    tidy();
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
        float z = (1 - t) * pStart[i][2] + t * target[2] + computeZOffset(static_cast<LegID>(i), x, y);

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
        float z = bodyHeight + computeZOffset(static_cast<LegID>(i), x, y);

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

void Robot::startup() {
    for (int i = 0; i < 4; i++) {
        legs[i].setAngles(0.7854f, 1.309f, 0);
        for (int j = 1; j < 11; j++) {
            legs[i].setAngles(0.7854f, 1.309f, j * 0.2407f);
            std::this_thread::sleep_for(std::chrono::milliseconds(STANDARD_DELAY));
        }
        for (int j = 1; j < 11; j++) {
            legs[i].setAngles(0.7854f, j * 0.1048f, 2.407f);
            std::this_thread::sleep_for(std::chrono::milliseconds(STANDARD_DELAY));
        }
    }
    bodyHeight = -120.0f;
    rest();
}

void Robot::tidy() {
    bodyHeight = -120.0f;
    rest();
    for (int i = 0; i < 4; i++) {
        for (int j = 1; j < 11; j++) {
            legs[i].setAngles(0.7854f, j * 0.1309f, 2.407f);
            std::this_thread::sleep_for(std::chrono::milliseconds(STANDARD_DELAY));
        }
    }
}

// Standard stance
void Robot::rest() {
    std::array<float, 3> p = {RESTING_X, RESTING_Y, bodyHeight};
    std::vector<std::pair<LegID, std::array<float, 3>>> targets = {{LegID::FL, p}, {LegID::FR, p}, {LegID::RR, p}, {LegID::RL, p}};
    moveLegs({}, targets, false, 0, 2);
}

void Robot::hi() {
    // paramètres (ajuste si besoin)
    const float tiltZ = 60.0f;      // combien on baisse FR / monte RL pour l'inclinaison
    const float liftZ  = 60.0f;     // élévation finale du pied FR pendant le 'lever'
    const float arcH   = 30.0f;     // hauteur d'arc pour l'élévation (pour donner une courbe)
    const int   stepsTilt = 40;     // interpolation pour l'inclinaison
    const int   stepsLift = 40;     // interpolation pour lever FR
    const int   stepsRotate = 12;   // pas pour tourner l'articulation
    const auto  sleepMs = std::chrono::milliseconds(STANDARD_DELAY);

    // 1) lire positions actuelles
    auto pos = getLegsPositions();
    // indices: 0=FL, 1=FR, 2=RR, 3=RL
    auto fr = pos[1];
    auto rl = pos[3];

    // 2) Incliner : baisser FR et lever RL
    std::array<float,3> frTilt = fr;
    std::array<float,3> rlTilt = rl;
    frTilt[2] -= tiltZ;   // abaisser FR
    rlTilt[2] += tiltZ;   // lever RL

    std::vector<std::pair<LegID, std::array<float,3>>> tiltTargets = {
        {LegID::FL, pos[0]},
        {LegID::FR, frTilt},
        {LegID::RR, pos[2]},
        {LegID::RL, rlTilt}
    };
    // IMPORTANT: transformTarget = false parce que pos provient de getLegsPositions()
    moveLegs({}, tiltTargets, false, 0.0f, stepsTilt);

    // 3) Lever FR (faire un arc puis placer le pied plus haut)
    auto posAfterTilt = getLegsPositions();
    auto frAfter = posAfterTilt[1];

    // Target final (FR plus haut)
    std::array<float,3> frLiftTarget = frAfter;
    frLiftTarget[2] += liftZ;

    // On met FR dans targetsLifted pour avoir un arc (liftHeight = arcH)
    moveLegs({{LegID::FR, frLiftTarget}}, {}, false, arcH, stepsLift);

    // 4) Tourner progressivement le dernier axe (joint 3) du FR
    auto anglesStart = legs[1].getAngles();
    float startA3 = anglesStart[2];
    float peakA3  = startA3 - (M_PIf / 2.0f); // tourner de +90° (1.5708)
    // monter à peak
    for (int s = 1; s <= stepsRotate; ++s) {
        float t = static_cast<float>(s) / stepsRotate;
        float a3 = startA3 + (peakA3 - startA3) * t;
        legs[1].setAngles(anglesStart[0], anglesStart[1], a3);
        std::this_thread::sleep_for(sleepMs);
    }
    // redescendre à startA3
    for (int s = 1; s <= stepsRotate; ++s) {
        float t = static_cast<float>(s) / stepsRotate;
        float a3 = peakA3 + (startA3 - peakA3) * t;
        legs[1].setAngles(anglesStart[0], anglesStart[1], a3);
        std::this_thread::sleep_for(sleepMs);
    }

    // 5) Rétablir FR à sa position après inclinaison (descendre le pied levé)
    // on ramène le FR au z d'avant levée (posAfterTilt[1][2])
    std::array<float,3> frBack = frAfter; // z = posAfterTilt[1][2]
    moveLegs({}, {{LegID::FL, posAfterTilt[0]}, {LegID::FR, frBack}, {LegID::RR, posAfterTilt[2]}, {LegID::RL, posAfterTilt[3]}}, false, 0.0f, stepsLift);

    // 6) Remettre tout à la position initiale (pos au début de hi)
    moveLegs({}, {{LegID::FL, pos[0]}, {LegID::FR, pos[1]}, {LegID::RR, pos[2]}, {LegID::RL, pos[3]}}, false, 0.0f, stepsTilt);
}

// Walk forward
/*
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
*/

void Robot::run(float x, float y) {
    const float h = bodyHeight;
    constexpr float dx = RUNNING_LEG_DISTANCE_FROM_BODY;   // Distance from body to end of leg on the sides
    const float mag = std::sqrt(x * x + y * y); // Magnitude of direction vector
    x = x / mag; y = y / mag; // Normalize vector
    //const float magu = std::sqrt(x * x + y * y); // Magnitude of normalized direction vector
    const float step = mag * runningStepSize;  // Distance a steps adds
    constexpr float stepHeight = RUNNING_STEP_HEIGHT;
    constexpr int pointsPerMovement = RUNNING_POINTS_PER_MOVEMENT;

    // // Initial position
    // std::vector<std::pair<LegID, std::array<float, 3>>> targetFlats = {
    //     {LegID::FL, {dx, dx, h}}, // FL
    //     {LegID::FR, {dx, dx, h}}, // FR
    //     {LegID::RR, {dx, dx, h}}, // RR
    //     {LegID::RL, {dx, dx, h}}  // RL
    // };

        moveLegs({{LegID::FL, {dx + step * x, dx + step * y, h}}, {LegID::RR, {dx - step * x, dx - step * y, h}}},
                 {{LegID::FR, {dx + step * y, dx - step * x, h}}, {LegID::RL, {dx - step * y, dx + step * x, h}}}, 0, stepHeight, pointsPerMovement);
//        moveLegs({{LegID::FR, {dx, dx, h}}, {LegID::RL, {dx, dx, h}}},
//                 {{LegID::FL, {dx, dx, h}}, {LegID::RR, {dx, dx, h}}}, 0, stepHeight, pointsPerMovement);
        moveLegs({{LegID::FR, {dx - step * y, dx + step * x, h}}, {LegID::RL, {dx + step * y, dx - step * x, h}}},
                 {{LegID::FL, {dx - step * x, dx - step * y, h}}, {LegID::RR, {dx + step * x, dx + step * y, h}}}, 0, stepHeight, pointsPerMovement);
//        moveLegs({{LegID::FL, {dx, dx, h}}, {LegID::RR, {dx, dx, h}}},
//                 {{LegID::FR, {dx, dx, h}}, {LegID::RL, {dx, dx, h}}}, 0, stepHeight, pointsPerMovement);
}

void Robot::stopRunning() {
    const float h = bodyHeight;
    constexpr float dx = RUNNING_LEG_DISTANCE_FROM_BODY;   // Distance from body to end of leg on the sides
    constexpr float stepHeight = RUNNING_STEP_HEIGHT;
    constexpr int pointsPerMovement = RUNNING_POINTS_PER_MOVEMENT;

    moveLegs({{LegID::FL, {dx, dx, h}}, {LegID::RR, {dx, dx, h}}},
             {{LegID::FR, {dx, dx, h}}, {LegID::RL, {dx, dx, h}}}, 0, stepHeight, pointsPerMovement);
}

// Turn left
void Robot::turn(bool left) {
    const float angle = left ? turningStepAngle : -turningStepAngle;

    //rest();
    std::vector<LegID> a = {LegID::FL, LegID::RR};
    std::vector<LegID> b = {LegID::FR, LegID::RL};
    rotateLegs(a, b, angle); // FL and RR lifted and going clockwise
    rotateLegs(b, a, angle); // FR and RL lifted and going clockwise
}

void Robot::level() {
    if (!imu) throw std::runtime_error("IMU not initialized");
    if (!stabilizer) throw std::runtime_error("Stabilizer not initialized");

    auto euler = imu->getEuler();
    auto positions = getLegsPositions();

    auto now = std::chrono::steady_clock::now();
    float dt = std::chrono::duration<float>(now - lastUpdate).count();
    if (dt <= 0) dt = 0.01f;
    lastUpdate = now;

    stabilizer->computeOffsets(euler[2], euler[1] - pitch, dt, positions);

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

void Robot::setBodyHeight(float newHeight) { bodyHeight = std::clamp(newHeight, -220.0f, -120.0f); }
void Robot::setRunningStepSize(float newSize) { runningStepSize = std::clamp(newSize, 20.0f, 60.0f); }
void Robot::setTurningStepAngle(float newAngle) { turningStepAngle = std::clamp(newAngle, 5.0f, 30.0f); }

void Robot::setPitch(float angleDeg) { pitch = std::clamp(angleDeg, -20.0f, 20.0f); }

float Robot::computeZOffset(LegID leg, float x, float y) {
    // angleDeg > 0: head up, angleDeg < 0: head down
    const float angleRad = pitch * M_PIf / 180.0f;
    const float tanAngle = std::tan(angleRad);
    int i = static_cast<int>(leg);

    switch (i) {
        case 0: break; // No necessary transformation
        case 1: x = y; break; // FR : exchanging x and y
        case 2: x = -x; break; // RR : inversing x
        case 3: x = -y; break; // RL : inversing y
    }

    return (i < 2) ? tanAngle * (x + CHASSIS * 0.5f) : -tanAngle * (x + CHASSIS * 0.5f);
}
