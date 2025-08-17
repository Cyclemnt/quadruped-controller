#ifndef ROBOT_HPP
#define ROBOT_HPP

#include "leg.hpp"
#include "pca9685.hpp"
#include <array>
#include <vector>

#define STANDARD_HEIGHT -170.0f
#define STANDARD_HEIGHT_2 -90.0f
#define STANDARD_DELAY 20

class Robot {
private:
    std::array<Leg, 4> legs;

public:
    Robot(PCA9685* driver);
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

    std::array<float, 3> getLegPosition(LegID id) const;
    std::array<std::array<float, 3>, 4> getLegsPositions() const;
};

#endif // ROBOT_HPP
