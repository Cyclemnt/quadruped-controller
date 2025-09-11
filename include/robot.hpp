#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "leg.hpp"
#include "pca9685.hpp"
#include "bno055.hpp"
#include "constants.hpp"
#include <array>
#include <vector>
#include <chrono>

struct PID {
    float kp, ki, kd;
    float integral;
    float prevError;

    PID(float p=1.0f, float i=0.0f, float d=0.0f)
        : kp(p), ki(i), kd(d), integral(0), prevError(0) {}

    float update(float error, float dt) {
        integral += error * dt;
        float derivative = (error - prevError) / dt;
        prevError = error;
        return kp * error + ki * integral + kd * derivative;
    }
};

class Robot {
private:
    std::array<Leg, 4> legs;
    BNO055* imu;

    PID pidRoll{LEVELING_P, LEVELING_I, LEVELING_D};   // valeurs à tuner
    PID pidPitch{LEVELING_P, LEVELING_I, LEVELING_D};

    std::chrono::steady_clock::time_point lastUpdate;

public:
    Robot(PCA9685* driver, BNO055* imu_ = nullptr);
    ~Robot();

    void transformXY(LegID id, float& x, float& y) const;

    void moveLegs(
        std::vector<std::pair<LegID, std::array<float, 3>>> targetsLifted,  // Containing legs to lift to target position
        std::vector<std::pair<LegID, std::array<float, 3>>> targetsFlat,    // Containing legs not to lift to target position
        bool transformTarget = 1, float liftHeight = 60.0f, int steps = 40
    );
    void rotateLegs(
        std::vector<LegID>& legsLifted,
        std::vector<LegID>& legsFlat,
        float deltaAngleDeg,
        float liftHeight = 20.0f, int steps = 40
    );

    void resetLegs();
    void rest();

    void sit(bool down);
    void walk();
    void run();
    void turn();

    float normalizeAngle(float angle);
    void level();

    std::array<float, 3> getLegPosition(LegID id) const;
    std::array<std::array<float, 3>, 4> getLegsPositions() const;
};

#endif // ROBOT_HPP
