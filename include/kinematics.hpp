#ifndef KINEMATICS_HPP
#define KINEMATICS_HPP

#include <array>

#define COXA 60.0f
#define FEMUR 100.0f
#define TIBIA 163.0f

class Kinematics {
public:
    static std::array<float, 3> computeFK(float q1, float q2, float q3);
    static std::array<float, 3> computeIK(float x, float y, float z);
};

#endif // KINEMATICS_HPP
