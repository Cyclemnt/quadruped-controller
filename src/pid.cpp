#include "../include/pid.hpp"

PID::PID(float kp_, float ki_, float kd_)
    : kp(kp_), ki(ki_), kd(kd_), integral(0.0f), prevError(0.0f) {}

PID::~PID() {}

float PID::update(float error, float dt) {
    integral += error * dt;
    float derivative = (dt > 0.0f) ? (error - prevError) / dt : 0.0f;
    prevError = error;
    return kp * error + ki * integral + kd * derivative;
}

void PID::reset() {
    integral = 0.0f;
    prevError = 0.0f;
}