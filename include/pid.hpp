#ifndef PID_HPP
#define PID_HPP

class PID {
private:
    float kp, ki, kd;

    float integral;
    float prevError;

public:
    PID(float kp = 0, float ki = 0, float kd = 0);
    ~PID();

    float update(float error, float dt);
    void reset();
};

#endif // PID_HPP
