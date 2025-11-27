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
#include <cmath>

class Robot {
private:
    std::array<Leg, 4> legs;
    BNO055* imu;

    Stabilizer* stabilizer;

    std::chrono::steady_clock::time_point lastUpdate;

    float bodyHeight, runningStepSize;//, turningStepAngle;
    std::array<float, 4> zOffset;
    float pitch; 


    // For vision rotation
    float chassisYawDeg = 0.0f;   // rotation autour de Z (deg)
    float chassisPitchDeg = 0.0f; // rotation autour de Y (deg) - même sens que setPitch
    float chassisRollDeg = 0.0f;  // rotation autour de X (deg)
    float chassisX = 0.0f;        // translation X du châssis (mm) si besoin
    float chassisY = 0.0f;        // translation Y du châssis (mm) si besoin

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
    void startup();
    void tidy();
    void rest();

    void hi();

    // void walk();
    void run(float x, float y);
    void stopRunning();
    void turn(float turningStepAngle = 15.0f);

    void level();

    std::array<float, 3> getLegPosition(LegID id) const;
    std::array<std::array<float, 3>, 4> getLegsPositions() const;

    void setBodyHeight(float newHeight);
    void setRunningStepSize(float newSize);
    // void setTurningStepAngle(float newAngle);
    void setPitch(float angleDeg);
    float computeZOffset(LegID leg, float x, float y);


    void orientChassisTo(float targetYawDeg, float targetPitchDeg, float targetRollDeg,
                            float targetX = 0.0f, float targetY = 0.0f, float targetZ = NAN,
                            int steps = 20);
    void lookAround(float jx, float jy, float maxYawDeg = 20.0f, float maxPitchDeg = 12.0f, int steps = 8);
    // void lookAroundInstant(float jx, float jy, float maxYawDeg, float maxPitchDeg);
};

#endif // ROBOT_HPP
