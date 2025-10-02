#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "leg.hpp"
#include "pca9685.hpp"
#include "bno055.hpp"
#include "constants.hpp"
#include "stabilizer.hpp"
#include "pid.hpp"
#include <array>
#include <vector>
#include <chrono>

class Robot {
private:
    std::array<Leg, 4> legs;
    BNO055* imu;

    Stabilizer* stabilizer;

    std::chrono::steady_clock::time_point lastUpdate;

    float bodyHeight, runningStepSize, turningStepAngle;

public:
    Robot(PCA9685* driver, BNO055* imu_ = nullptr, Stabilizer* stabilizer_ = nullptr);
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
        float liftHeight = 30.0f, int steps = 8
    );

    void resetLegs();
    void rest();

    void sit(bool down);
    void walk();
    void run(bool frontwards);
    void stopRunning();
    void turn(bool left);

    void level();

    std::array<float, 3> getLegPosition(LegID id) const;
    std::array<std::array<float, 3>, 4> getLegsPositions() const;

    void setBodyHeight(float newHeight);
    void setRunningStepSize(float newSize);
    void setTurningStepAngle(float newAngle);
};

#endif // ROBOT_HPP
