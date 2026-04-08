#include <Arduino.h>
#include <math.h>
#include "SystemState.h"
#include "PIDController.h"
#include "SensorManager.h"
#include "SerialProtocol.h"
#include "ExtSensorRegistry.h"

// ── Pin definitions ────────────────────────────────────────────────────────────
static const int PIN_HEATER_MOSFET = 4;
static const int PIN_FAN_PWM       = 5;

// ── PWM channels ──────────────────────────────────────────────────────────────
static const int HEATER_PWM_CH  = 0;
static const int FAN_PWM_CH     = 1;
static const int PWM_FREQ       = 500;
static const int PWM_RESOLUTION = 8;

// ── Control timing ─────────────────────────────────────────────────────────────
static const uint32_t CONTROL_PERIOD_MS = 500;
static uint32_t lastControlMs = 0;

// ── Thermocouple accuracy constant (used in SMART mode) ──────────────────────
static const float THERMOCOUPLE_ACCURACY_C = 0.25f;

// ── SMART mode state ───────────────────────────────────────────────────────────
static const uint16_t SMART_PWM_AVG_WINDOW = 40;  // 20 s at 0.5 s period
static float    smartPwmHistory[SMART_PWM_AVG_WINDOW];
static uint16_t smartPwmHistCount = 0;
static uint16_t smartPwmHistIndex = 0;
static uint16_t smartStableCount  = 0;
static uint16_t smartDriftCount   = 0;
static float    lastSmoothForSlope = NAN;
static float    pwmDitherAccumulator = 0.0f;

// ── PWM slew-rate state ────────────────────────────────────────────────────────
static int lastPWM = 0;

// ── Global instances ───────────────────────────────────────────────────────────
SystemState     sys;
PIDController   pid(8.0f, 0.06f, 0.0f);

// (sensors and extSensors are defined in their respective .cpp files)

// ── System state defaults ──────────────────────────────────────────────────────
static void initSysDefaults() {
    sys.pidKp              = 8.0f;
    sys.pidKi              = 0.06f;
    sys.pidKd              = 0.0f;
    sys.pidBias            = 0.0f;
    sys.setpointBias       = 0.0f;
    sys.setpoint           = 70.0f;
    sys.emaAlpha           = 0.25f;
    sys.maxPwmStep         = 15;
    sys.fanSpeedPercent    = 0.0f;
    sys.fanPwmInverted     = false;
    sys.fanPwmApplied      = 255;
    sys.controlMode        = MODE_AUTO;
    sys.manualPwmTarget    = 0.0f;
    sys.controlEnabled     = true;
    sys.smartEnterCount    = 16;
    sys.smartExitCount     = 8;
    sys.smartHoldActive    = false;
    sys.smartHoldPwmTarget = 0.0f;
    sys.smartEnterProgressPct = 0.0f;
    sys.smartExitProgressPct  = 0.0f;
    sys.eqPwm              = 0.0f;
}

// ── Actuator helpers ───────────────────────────────────────────────────────────

static int fanPercentToRaw(float percent) {
    float clamped = constrain(percent, 0.0f, 100.0f);
    float duty = clamped / 100.0f;
    if (!sys.fanPwmInverted) duty = 1.0f - duty;
    return constrain((int)lroundf(255.0f * duty), 0, 255);
}

void setFanSpeedPercent(float percent) {
    sys.fanSpeedPercent = constrain(percent, 0.0f, 100.0f);
    int raw = fanPercentToRaw(sys.fanSpeedPercent);
    if (sys.controlEnabled) {
        ledcWrite(FAN_PWM_CH, raw);
        sys.fanPwmApplied = raw;
    } else {
        ledcWrite(FAN_PWM_CH, 0);
        sys.fanPwmApplied = 0;
    }
}

static void setPWM(int duty) {
    duty = constrain(duty, 0, 255);
    ledcWrite(HEATER_PWM_CH, duty);
}

// ── PID helpers ────────────────────────────────────────────────────────────────

void applyPidTunings() {
    pid.setTunings(sys.pidKp, sys.pidKi, sys.pidKd);
    pid.setBias(sys.pidBias);
    pid.reset();
}

// ── SMART mode helpers ─────────────────────────────────────────────────────────

void resetSmartState() {
    sys.smartHoldActive       = false;
    sys.smartHoldPwmTarget    = 0.0f;
    sys.smartEnterProgressPct = 0.0f;
    sys.smartExitProgressPct  = 0.0f;
    smartStableCount          = 0;
    smartDriftCount           = 0;
    smartPwmHistCount         = 0;
    smartPwmHistIndex         = 0;
    pwmDitherAccumulator      = 0.0f;
    lastSmoothForSlope        = NAN;
}

static void pushSmartPwmSample(float pwm) {
    smartPwmHistory[smartPwmHistIndex] = pwm;
    smartPwmHistIndex = (uint16_t)((smartPwmHistIndex + 1) % SMART_PWM_AVG_WINDOW);
    if (smartPwmHistCount < SMART_PWM_AVG_WINDOW) smartPwmHistCount++;
}

static float getSmartPwmAverage() {
    if (smartPwmHistCount == 0) return 0.0f;
    float sum = 0.0f;
    for (uint16_t i = 0; i < smartPwmHistCount; i++) sum += smartPwmHistory[i];
    return sum / (float)smartPwmHistCount;
}

static int ditheredPwmFromTarget(float targetPwm) {
    float clamped = constrain(targetPwm, 0.0f, 255.0f);
    int base = (int)floorf(clamped);
    pwmDitherAccumulator += clamped - (float)base;
    int pwm = base;
    if (pwmDitherAccumulator >= 1.0f) { pwm++; pwmDitherAccumulator -= 1.0f; }
    return constrain(pwm, 0, 255);
}

static int limitPWMChange(int pwm, int maxStep) {
    if (pwm > lastPWM + maxStep) pwm = lastPWM + maxStep;
    if (pwm < lastPWM - maxStep) pwm = lastPWM - maxStep;
    lastPWM = pwm;
    return pwm;
}

// ── setup ──────────────────────────────────────────────────────────────────────

void setup() {
    Serial.begin(115200);
    delay(500);

    initSysDefaults();
    SerialProtocol::loadConfig();  // overlay NVS values onto sys

    sensors.begin(sys.emaAlpha);

    ledcSetup(HEATER_PWM_CH, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PIN_HEATER_MOSFET, HEATER_PWM_CH);
    ledcSetup(FAN_PWM_CH, PWM_FREQ, PWM_RESOLUTION);
    ledcAttachPin(PIN_FAN_PWM, FAN_PWM_CH);

    setPWM(0);
    setFanSpeedPercent(sys.fanSpeedPercent);
    applyPidTunings();
    resetSmartState();

    // ── Register extension sensors here when hardware is connected ────────
    // Example (uncomment and adapt when sensor is wired):
    //   int8_t airspeedSlot = extSensors.registerSensor("AIRSPEED", "m/s");
    //   int8_t deltaP1Slot  = extSensors.registerSensor("DELTA_P1", "Pa");
    //   int8_t deltaP2Slot  = extSensors.registerSensor("DELTA_P2", "Pa");
    //   int8_t humiditySlot = extSensors.registerSensor("HUMIDITY", "%RH");
    // ─────────────────────────────────────────────────────────────────────

    Serial.println("System started");
    Serial.println("Commands: GET, SET KP <v>, SET KI <v>, SET KD <v>, SET BIAS <v>, SET SPBIAS <v>, SET SP <v>, SET ALPHA <v>, SET MAXSTEP <v>, SET ENTERCNT <1..400>, SET EXITCNT <1..400>, SET FAN <0..100>, SET FANINV <0|1>, SET MODE <AUTO|MANUAL|SMART>, SET MANPWM <0..255>, SET RUN <ON|OFF>");
    SerialProtocol::printConfig();
}

// ── loop ───────────────────────────────────────────────────────────────────────

void loop() {
    SerialProtocol::processCommands();

    uint32_t now = millis();
    if ((now - lastControlMs) < CONTROL_PERIOD_MS) return;
    lastControlMs = now;

    // ── Read all sensors ──────────────────────────────────────────────────
    CoreSensorData sd = sensors.read();

    // ── Update extension sensor slots here when hardware is connected ─────
    // Example:
    //   extSensors.update(airspeedSlot, readAirspeed());
    //   extSensors.update(deltaP1Slot,  readDeltaP1());
    // ─────────────────────────────────────────────────────────────────────

    // ── Thermocouple fault: cut heater immediately ─────────────────────────
    if (isnan(sd.filteredTemp)) {
        setPWM(0);
        ledcWrite(FAN_PWM_CH, 0);
        sys.fanPwmApplied = 0;
        pid.reset();
        resetSmartState();
        sensors.resetEma();
        Serial.println("Temp read error -> heater OFF (NaN/out of range)");
        return;
    }

    if (sd.tempStuck) {
        setPWM(0);
        ledcWrite(FAN_PWM_CH, 0);
        sys.fanPwmApplied = 0;
        pid.reset();
        resetSmartState();
        sensors.resetEma();
        sensors.resetStuck();
        Serial.println("Thermocouple STUCK detected -> heater OFF + PID reset");
        delay(500);
        return;
    }

    // ── Derived quantities ────────────────────────────────────────────────
    float effectiveSetpoint = constrain(sys.setpoint + sys.setpointBias, -20.0f, 400.0f);
    float error             = effectiveSetpoint - sd.smoothTemp;
    float absError          = fabsf(error);
    float absErrorMinusAcc  = max(0.0f, absError - THERMOCOUPLE_ACCURACY_C);

    float slope = 0.0f;
    if (!isnan(lastSmoothForSlope)) {
        slope = (sd.smoothTemp - lastSmoothForSlope) / ((float)CONTROL_PERIOD_MS / 1000.0f);
    }
    float absSlope = fabsf(slope);
    lastSmoothForSlope = sd.smoothTemp;

    const float smartEnterErrC   = THERMOCOUPLE_ACCURACY_C * 2.0f;  // 0.50 °C
    const float smartEnterSlopeCps = 0.25f;
    const float smartExitErrC    = THERMOCOUPLE_ACCURACY_C * 4.0f;  // 1.00 °C

    // ── Control law ───────────────────────────────────────────────────────
    int pwm = 0;

    if (!sys.controlEnabled) {
        setPWM(0);
        ledcWrite(FAN_PWM_CH, 0);
        sys.fanPwmApplied = 0;
        sys.smartHoldActive       = false;
        smartStableCount          = 0;
        smartDriftCount           = 0;
        sys.smartEnterProgressPct = 0.0f;
        sys.smartExitProgressPct  = 0.0f;
    } else {
        setFanSpeedPercent(sys.fanSpeedPercent);

        if (sys.controlMode == MODE_MANUAL) {
            pwm = ditheredPwmFromTarget(sys.manualPwmTarget);
            sys.smartHoldActive       = false;
            sys.smartEnterProgressPct = 0.0f;
            sys.smartExitProgressPct  = 0.0f;
            lastPWM = pwm;

        } else if (sys.controlMode == MODE_AUTO) {
            pwm = (int)pid.calculate(effectiveSetpoint, sd.smoothTemp);
            pwm = limitPWMChange(pwm, sys.maxPwmStep);
            sys.smartHoldActive       = false;
            smartStableCount          = 0;
            smartDriftCount           = 0;
            sys.smartEnterProgressPct = 0.0f;
            sys.smartExitProgressPct  = 0.0f;

        } else {
            // ── SMART mode ─────────────────────────────────────────────────
            if (!sys.smartHoldActive) {
                pwm = (int)pid.calculate(effectiveSetpoint, sd.smoothTemp);
                pwm = limitPWMChange(pwm, sys.maxPwmStep);
                pushSmartPwmSample((float)pwm);

                bool isStable = (absErrorMinusAcc <= smartEnterErrC) && (absSlope <= smartEnterSlopeCps);
                if (isStable) {
                    if (smartStableCount < sys.smartEnterCount) smartStableCount++;
                } else {
                    if (smartStableCount > 0) smartStableCount--;
                }
                sys.smartEnterProgressPct = min(100.0f, 100.0f * ((float)smartStableCount / (float)sys.smartEnterCount));
                sys.smartExitProgressPct  = 0.0f;

                if (smartStableCount >= sys.smartEnterCount && smartPwmHistCount >= 10) {
                    sys.smartHoldPwmTarget = getSmartPwmAverage();
                    sys.smartHoldActive    = true;
                    smartDriftCount        = 0;
                    pwmDitherAccumulator   = 0.0f;
                    sys.smartEnterProgressPct = 100.0f;
                    Serial.printf("SMART ENTER HOLD | HOLDPWM: %.2f | ERR: %.2f C | SLOPE: %.3f C/s\n",
                                  sys.smartHoldPwmTarget, error, slope);
                }
            } else {
                pwm = ditheredPwmFromTarget(sys.smartHoldPwmTarget);
                lastPWM = pwm;

                if (absErrorMinusAcc >= smartExitErrC) {
                    smartDriftCount++;
                } else {
                    smartDriftCount = 0;
                }
                sys.smartEnterProgressPct = 100.0f;
                sys.smartExitProgressPct  = min(100.0f, 100.0f * ((float)smartDriftCount / (float)sys.smartExitCount));

                if (smartDriftCount >= sys.smartExitCount) {
                    sys.smartHoldActive       = false;
                    smartStableCount          = 0;
                    smartDriftCount           = 0;
                    smartPwmHistCount         = 0;
                    smartPwmHistIndex         = 0;
                    sys.smartEnterProgressPct = 0.0f;
                    sys.smartExitProgressPct  = 0.0f;
                    pid.reset();
                    Serial.printf("SMART EXIT HOLD -> PID | ERR: %.2f C | SLOPE: %.3f C/s\n", error, slope);
                }
            }
        }

        setPWM(pwm);
    }

    // ── Equilibrium PWM estimate ──────────────────────────────────────────
    if (absSlope < 0.3f && sys.controlEnabled) {
        sys.eqPwm = 0.05f * (float)pwm + 0.95f * sys.eqPwm;
    }

    // ── Telemetry ─────────────────────────────────────────────────────────
    float pTerm = 0.0f, iTerm = 0.0f, dTerm = 0.0f, pidOut = 0.0f;
    pid.getLastTerms(pTerm, iTerm, dTerm, pidOut);

    const char *stateText = "PID";
    if (sys.controlMode == MODE_MANUAL) {
        stateText = "MANUAL";
    } else if (sys.controlMode == MODE_SMART && sys.smartHoldActive) {
        stateText = "HOLD";
    }

    SerialProtocol::emitTelemetry(sd, pwm, pTerm, iTerm, dTerm, pidOut, stateText);
}
