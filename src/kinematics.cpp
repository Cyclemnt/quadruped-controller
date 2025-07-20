#include "../include/kinematics.hpp"
#include <cmath>
#include <algorithm>
#include <iostream>

// Forward kinematics
std::array<float, 3> Kinematics::computeFK(float q1, float q2, float q3) {
    q2 = -q2;
    float x = cos(q1)*(COXA + FEMUR*cos(q2) + TIBIA*cos(q2 + q3));
    float y = sin(q1)*(COXA + FEMUR*cos(q2) + TIBIA*cos(q2 + q3));
    float z = - FEMUR*sin(q2) - TIBIA*sin(q2 + q3);
    return {x, y, z};
}

// Inverse kinematics
// Thanks to Robert Eisele for fast execution script :
// https://raw.org/book/robotics/inverse-kinematics-of-a-3-dof-spider-robot-leg/
std::array<float, 3> Kinematics::computeIK(float x, float y, float z) {
    // 1) Yaw about Z
    float q1 = atan2(y, x); // For math, M_PI_2 - q1 irl
    float q2 = 0, q3 = 0;

    // 2) Project into hip-pitch plane
    float dy = sqrt(x * x + y * y) - COXA;
    float dz = z;

    float v2 = dy * dy + dz * dz;

    // 3) Inner workspace: too close => knee fully bent
    if (v2 < (TIBIA - FEMUR) * (TIBIA - FEMUR)) {
        q2 = M_PIf + atan2(dz, dy);
        q3 = 0; // folded
        std::cout << "Too close" << std::endl;
        return {q1, -q2, M_PIf - q3};
    }

    // 4) Outer workspace: too far => leg fully extended
    if (v2 > (FEMUR + TIBIA) * (FEMUR + TIBIA)) {
        q2 = atan2(dz, dy);
        q3 = M_PIf;
        std::cout << "Too far" << std::endl;
        return {q1, -q2, M_PIf - q3};
    }

    // 5) Calculate discriminant
    float C = FEMUR * FEMUR - TIBIA * TIBIA + v2;
    float disc = 4 * FEMUR * FEMUR * v2 - C * C;
    float S = disc > 0 ? sqrt(disc) : 0;

    // 6) Femur pitch
    q2 = atan2(dy * S + dz * C, dy * C - dz * S);

    // 7) Tibia pitch
    q3 = acos((FEMUR * FEMUR + TIBIA * TIBIA - v2) / (2 * FEMUR * TIBIA));

    
    return {q1, q2, M_PIf - q3};
}
