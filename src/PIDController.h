#ifndef PIDCONTROLLER_H
#define PIDCONTROLLER_H

#include <Arduino.h>

class PIDController {
private:
    float kp, ki, kd;
    float bias;
    float integral;
    float lastError;
    unsigned long lastTime;
    float lastPTerm;
    float lastITerm;
    float lastDTerm;
    float lastOutput;

public:
    PIDController(float kp_, float ki_, float kd_);

    void reset();
    float calculate(float setpoint, float measured);
    void setTunings(float kp_, float ki_, float kd_);
    void getTunings(float &kp_, float &ki_, float &kd_) const;
    void setBias(float bias_);
    float getBias() const;
    void getLastTerms(float &pTerm, float &iTerm, float &dTerm, float &output) const;
};

#endif
