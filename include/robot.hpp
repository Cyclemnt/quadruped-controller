#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "leg.hpp"
#include "pca9685.hpp"
#include "bno055.hpp"
#include <array>
#include <vector>

#define STANDARD_HEIGHT -170.0f
#define STANDARD_HEIGHT_2 -90.0f
#define STANDARD_DELAY 20

class Robot {
private:
    std::array<Leg, 4> legs;
    BNO055* imu;

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

    std::array<float, 3> getOrientation(); // heading, roll, pitch

    std::array<float, 3> getLegPosition(LegID id) const;
    std::array<std::array<float, 3>, 4> getLegsPositions() const;
};

#endif // ROBOT_HPP
