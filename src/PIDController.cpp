#include "PIDController.h"

PIDController::PIDController(float kp_, float ki_, float kd_)
    : kp(kp_), ki(ki_), kd(kd_), bias(0.0f), integral(0.0f), lastError(0.0f), lastTime(0),
      lastPTerm(0.0f), lastITerm(0.0f), lastDTerm(0.0f), lastOutput(0.0f) {}

void PIDController::reset() {
    integral = 0.0f;
    lastError = 0.0f;
    lastTime = millis();
    lastPTerm = 0.0f;
    lastITerm = 0.0f;
    lastDTerm = 0.0f;
    lastOutput = 0.0f;
}

float PIDController::calculate(float setpoint, float measured) {
    unsigned long now = millis();
    float dt = (now - lastTime) / 1000.0f;
    if (dt <= 0) dt = 0.001f;
    lastTime = now;

    float error = setpoint - measured;
    float derivative = (error - lastError) / dt;
    lastError = error;

    float pTerm = kp * error;
    float dTerm = kd * derivative;

    // Candidate integrator update before anti-windup decision.
    float integralCandidate = integral + error * dt;

    // Keep I-term capability consistent across Ki values.
    float integralLimit = 10000.0f;
    float absKi = fabsf(ki);
    if (absKi > 1e-6f) {
        integralLimit = 255.0f / absKi;
    }
    integralCandidate = constrain(integralCandidate, -integralLimit, integralLimit);
    float iTermCandidate = ki * integralCandidate;

    float unsatOutput = pTerm + iTermCandidate + dTerm + bias;

    // Conditional integration anti-windup:
    // do not integrate if output is saturated and error pushes further into saturation.
    bool saturatingHigh = (unsatOutput > 255.0f) && (error > 0.0f);
    bool saturatingLow = (unsatOutput < 0.0f) && (error < 0.0f);
    if (!(saturatingHigh || saturatingLow)) {
        integral = integralCandidate;
    }

    float iTerm = ki * integral;
    float output = pTerm + iTerm + dTerm + bias;
    float constrainedOutput = constrain(output, 0.0f, 255.0f);

    lastPTerm = pTerm;
    lastITerm = iTerm;
    lastDTerm = dTerm;
    lastOutput = constrainedOutput;

    return constrainedOutput;
}

void PIDController::setTunings(float kp_, float ki_, float kd_) {
    kp = kp_;
    ki = ki_;
    kd = kd_;
}

void PIDController::getTunings(float &kp_, float &ki_, float &kd_) const {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
}

void PIDController::setBias(float bias_) {
    bias = bias_;
}

float PIDController::getBias() const {
    return bias;
}

void PIDController::getLastTerms(float &pTerm, float &iTerm, float &dTerm, float &output) const {
    pTerm = lastPTerm;
    iTerm = lastITerm;
    dTerm = lastDTerm;
    output = lastOutput;
}
