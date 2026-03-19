#include <Arduino.h>
#include "max6675.h"
#include "PIDController.h"
#include <Preferences.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <Wire.h>
#include "INA226.h"

// ---------------- PIN DEFINITIES ----------------
const int heaterMosfetPin = 4;
const int fanPwmPin = 5;

const int thermocoupleSCK = 13;
const int thermocoupleCS  = 12;
const int thermocoupleSO  = 11;

const int i2cSdaPin = 15;
const int i2cSclPin = 16;

// ---------------- PWM INSTELLINGEN ----------------
const int heaterPwmChannel = 0;
const int fanPwmChannel    = 1;
const int pwmFreq = 500;
const int pwmResolution = 8;

// ---------------- OBJECTEN ----------------
MAX6675 thermocouple(thermocoupleSCK, thermocoupleCS, thermocoupleSO);
INA226 ina226(0x40);
float pidKp = 8.0f;
float pidKi = 0.06f;
float pidKd = 0.0f;
float pidBias = 0.0f;
float setpointBias = 0.0f;
PIDController pid(pidKp, pidKi, pidKd);
Preferences preferences;

// ---------------- CONTROL TIMING (millis) ----------------
const uint32_t controlPeriodMs = 500;
uint32_t lastControlMs = 0;

// ---------------- SETPOINT ----------------
float setpoint = 70.0f;
float fanSpeedPercent = 0.0f;
int fanPwmRaw = 255;
float emaAlpha = 0.25f;
int maxPwmStep = 15;

enum ControlMode {
  MODE_AUTO = 0,
  MODE_MANUAL = 1,
  MODE_SMART = 2
};

ControlMode controlMode = MODE_AUTO;
float manualPwmTarget = 0.0f;

bool smartHoldActive = false;
float smartHoldPwmTarget = 0.0f;
float pwmDitherAccumulator = 0.0f;
bool fanPwmInverted = false;
bool controlEnabled = true;

const float thermocoupleAccuracyC = 0.25f;
const float smartEnterErrC = thermocoupleAccuracyC * 2.0f; // 0.50 C
const float smartEnterSlopeCps = 0.25f;
uint16_t smartEnterCount = 16;         // 16 * 0.5s = 8s effective stable window
const float smartExitErrC = thermocoupleAccuracyC * 4.0f;  // 1.00 C
uint16_t smartExitCount = 8;           // 8 * 0.5s = 4s outside band before fallback

const uint16_t smartPwmAvgWindow = 40; // 20s average at 0.5s sample period
float smartPwmHistory[smartPwmAvgWindow];
uint16_t smartPwmHistCount = 0;
uint16_t smartPwmHistIndex = 0;
uint16_t smartStableCount = 0;
uint16_t smartDriftCount = 0;
float lastSmoothForSlope = NAN;
float smartEnterProgressPct = 0.0f;
float smartExitProgressPct = 0.0f;
int fanPwmApplied = 255;
float eqPwm = 0.0f;  // Converging equilibrium PWM estimate

// ---------------- INA226 METINGEN ----------------
float supplyVoltage = 12.0f; // Set to your PSU voltage
float inaVoltage = 0.0f;   // Bus voltage in V
float inaCurrent = 0.0f;   // Current in A
float inaPower   = 0.0f;   // Power in W
bool  inaOk      = false;

void saveConfigToNvs();
void loadConfigFromNvs();

const char *modeToText(ControlMode mode);
bool parseMode(const char *text, ControlMode &mode);
void resetSmartState();
void pushSmartPwmSample(float pwm);
float getSmartPwmAverage();
int ditheredPwmFromTarget(float targetPwm);

// ---------------- FUNCTIES ----------------
void setPWM(int duty)
{
  duty = constrain(duty, 0, 255);
  ledcWrite(heaterPwmChannel, duty);
}

int fanPercentToRaw(float percent) {
  float clamped = constrain(percent, 0.0f, 100.0f);
  float duty = clamped / 100.0f;
  if (!fanPwmInverted) {
    duty = 1.0f - duty;
  }
  int raw = (int)lroundf(255.0f * duty);
  return constrain(raw, 0, 255);
}

void setFanSpeedPercent(float percent) {
  fanSpeedPercent = constrain(percent, 0.0f, 100.0f);
  fanPwmRaw = fanPercentToRaw(fanSpeedPercent);
  if (controlEnabled) {
    ledcWrite(fanPwmChannel, fanPwmRaw);
    fanPwmApplied = fanPwmRaw;
  } else {
    ledcWrite(fanPwmChannel, 0);
    fanPwmApplied = 0;
  }
}

void saveConfigToNvs() {
  preferences.putFloat("kp", pidKp);
  preferences.putFloat("ki", pidKi);
  preferences.putFloat("kd", pidKd);
  preferences.putFloat("bias", pidBias);
  preferences.putFloat("spbias", setpointBias);
  preferences.putFloat("sp", setpoint);
  preferences.putFloat("alpha", emaAlpha);
  preferences.putInt("maxstep", maxPwmStep);
  preferences.putInt("entcnt", smartEnterCount);
  preferences.putInt("extcnt", smartExitCount);
  preferences.putFloat("fanpct", fanSpeedPercent);
  preferences.putBool("faninv", fanPwmInverted);
  preferences.putInt("mode", (int)controlMode);
  preferences.putFloat("manpwmf", manualPwmTarget);
  // Backward-compat keys
  preferences.putBool("manual", controlMode == MODE_MANUAL);
  preferences.putInt("manpwm", (int)lroundf(manualPwmTarget));
}

void loadConfigFromNvs() {
  pidKp = preferences.getFloat("kp", pidKp);
  pidKi = preferences.getFloat("ki", pidKi);
  pidKd = preferences.getFloat("kd", pidKd);
  pidBias = constrain(preferences.getFloat("bias", pidBias), -255.0f, 255.0f);
  setpointBias = constrain(preferences.getFloat("spbias", setpointBias), -200.0f, 200.0f);
  setpoint = constrain(preferences.getFloat("sp", setpoint), -20.0f, 400.0f);
  emaAlpha = constrain(preferences.getFloat("alpha", emaAlpha), 0.001f, 1.0f);
  maxPwmStep = constrain(preferences.getInt("maxstep", maxPwmStep), 0, 255);
  smartEnterCount = constrain(preferences.getInt("entcnt", smartEnterCount), 1, 400);
  smartExitCount = constrain(preferences.getInt("extcnt", smartExitCount), 1, 400);
  fanSpeedPercent = constrain(preferences.getFloat("fanpct", fanSpeedPercent), 0.0f, 100.0f);
  fanPwmInverted = preferences.getBool("faninv", fanPwmInverted);
  if (preferences.isKey("mode")) {
    int modeRaw = preferences.getInt("mode", (int)MODE_AUTO);
    if (modeRaw < (int)MODE_AUTO || modeRaw > (int)MODE_SMART) modeRaw = (int)MODE_AUTO;
    controlMode = (ControlMode)modeRaw;
  } else {
    // Backward compatibility with old stored config.
    bool manual = preferences.getBool("manual", false);
    controlMode = manual ? MODE_MANUAL : MODE_AUTO;
  }

  if (preferences.isKey("manpwmf")) {
    manualPwmTarget = constrain(preferences.getFloat("manpwmf", manualPwmTarget), 0.0f, 255.0f);
  } else {
    manualPwmTarget = constrain((float)preferences.getInt("manpwm", (int)manualPwmTarget), 0.0f, 255.0f);
  }
}

const char *modeToText(ControlMode mode) {
  switch (mode) {
    case MODE_MANUAL: return "MANUAL";
    case MODE_SMART: return "SMART";
    case MODE_AUTO:
    default: return "AUTO";
  }
}

bool parseMode(const char *text, ControlMode &mode) {
  if (strcmp(text, "AUTO") == 0) {
    mode = MODE_AUTO;
    return true;
  }
  if (strcmp(text, "MANUAL") == 0) {
    mode = MODE_MANUAL;
    return true;
  }
  if (strcmp(text, "SMART") == 0) {
    mode = MODE_SMART;
    return true;
  }
  return false;
}

void resetSmartState() {
  smartHoldActive = false;
  smartHoldPwmTarget = 0.0f;
  smartStableCount = 0;
  smartDriftCount = 0;
  smartPwmHistCount = 0;
  smartPwmHistIndex = 0;
  pwmDitherAccumulator = 0.0f;
  lastSmoothForSlope = NAN;
  smartEnterProgressPct = 0.0f;
  smartExitProgressPct = 0.0f;
}

void pushSmartPwmSample(float pwm) {
  smartPwmHistory[smartPwmHistIndex] = pwm;
  smartPwmHistIndex = (uint16_t)((smartPwmHistIndex + 1) % smartPwmAvgWindow);
  if (smartPwmHistCount < smartPwmAvgWindow) {
    smartPwmHistCount++;
  }
}

float getSmartPwmAverage() {
  if (smartPwmHistCount == 0) return 0.0f;
  float sum = 0.0f;
  for (uint16_t i = 0; i < smartPwmHistCount; i++) {
    sum += smartPwmHistory[i];
  }
  return sum / (float)smartPwmHistCount;
}

int ditheredPwmFromTarget(float targetPwm) {
  float clamped = constrain(targetPwm, 0.0f, 255.0f);
  int base = (int)floorf(clamped);
  float frac = clamped - (float)base;
  pwmDitherAccumulator += frac;
  int pwm = base;
  if (pwmDitherAccumulator >= 1.0f) {
    pwm += 1;
    pwmDitherAccumulator -= 1.0f;
  }
  return constrain(pwm, 0, 255);
}

// -------- Glitch reject --------
float lastGoodTemp = NAN;

bool isValidTemp(float t) {
  if (isnan(t)) return false;
  if (t < -20 || t > 400) return false;
  return true;
}

float readTempFiltered(float t) {
  if (!isValidTemp(t)) return NAN;

  if (!isnan(lastGoodTemp)) {
    float maxJump = 100.0f;
    if (fabs(t - lastGoodTemp) > maxJump) {
      return lastGoodTemp;
    }
  }

  lastGoodTemp = t;
  return t;
}

// -------- EMA smoothing --------
float emaTemp = NAN;

float smoothTemp(float t) {
  if (isnan(emaTemp)) emaTemp = t;
  emaTemp = emaAlpha * t + (1.0f - emaAlpha) * emaTemp;
  return emaTemp;
}

// -------- PWM slew-rate limit --------
int lastPWM = 0;

int limitPWMChange(int pwm, int maxStep) {
  if (pwm > lastPWM + maxStep) pwm = lastPWM + maxStep;
  if (pwm < lastPWM - maxStep) pwm = lastPWM - maxStep;
  lastPWM = pwm;
  return pwm;
}

// -------- Stuck watchdog --------
float lastTempForStuck = NAN;
uint16_t sameCount = 0;
const float stuckEpsilon = 0.01f;
const uint16_t stuckLimit = 30;

bool stuckDetected(float t) {
  if (isnan(lastTempForStuck)) {
    lastTempForStuck = t;
    sameCount = 0;
    return false;
  }

  if (fabs(t - lastTempForStuck) < stuckEpsilon) {
    sameCount++;
  } else {
    sameCount = 0;
    lastTempForStuck = t;
  }

  return (sameCount >= stuckLimit);
}

// -------- Runtime commands over serial --------
const size_t commandBufferSize = 96;
char commandBuffer[commandBufferSize];
size_t commandLen = 0;

void printConfig() {
  float kp;
  float ki;
  float kd;
  pid.getTunings(kp, ki, kd);
  Serial.printf("CFG KP: %.3f | KI: %.3f | KD: %.3f | BIAS: %.2f | SPBIAS: %.2f | SP: %.2f | ALPHA: %.3f | MAXSTEP: %d | ENTCNT: %d | EXTCNT: %d | FAN: %.1f | FANINV: %d | MODE: %s | MANPWM: %.2f | RUN: %s\n",
                kp, ki, kd, pid.getBias(), setpointBias, setpoint, emaAlpha, maxPwmStep, smartEnterCount, smartExitCount,
                fanSpeedPercent, fanPwmInverted ? 1 : 0, modeToText(controlMode), manualPwmTarget, controlEnabled ? "ON" : "OFF");
}

void applyPidTunings() {
  pid.setTunings(pidKp, pidKi, pidKd);
  pid.setBias(pidBias);
  pid.reset();
}

void handleCommand(char *line) {
  while (*line == ' ' || *line == '\t') {
    line++;
  }

  if (*line == '\0') {
    return;
  }

  if (strcmp(line, "GET") == 0) {
    printConfig();
    return;
  }

  char *command = strtok(line, " \t");
  if (!command || strcmp(command, "SET") != 0) {
    Serial.println("ERR Unknown command. Use SET <KP|KI|KD|BIAS|SPBIAS|SP|ALPHA|MAXSTEP|ENTERCNT|EXITCNT|FAN|FANINV|MODE|MANPWM|RUN> <value> or GET");
    return;
  }

  char *key = strtok(NULL, " \t");
  char *valueText = strtok(NULL, " \t");
  if (!key || !valueText) {
    Serial.println("ERR Usage: SET <KP|KI|KD|BIAS|SPBIAS|SP|ALPHA|MAXSTEP|ENTERCNT|EXITCNT|FAN|FANINV|MODE|MANPWM|RUN> <value>");
    return;
  }

  if (strcmp(key, "MAXSTEP") == 0) {
    int valueInt = atoi(valueText);
    if (valueInt < 0 || valueInt > 255) {
      Serial.println("ERR MAXSTEP must be in range 0..255");
      return;
    }
    maxPwmStep = valueInt;
    Serial.printf("OK MAXSTEP set to %d\n", maxPwmStep);
    saveConfigToNvs();
    printConfig();
    return;
  }

  if (strcmp(key, "MODE") == 0) {
    ControlMode newMode = MODE_AUTO;
    if (!parseMode(valueText, newMode)) {
      Serial.println("ERR MODE must be AUTO, MANUAL, or SMART");
      return;
    }
    controlMode = newMode;
    pid.reset();
    resetSmartState();
    Serial.printf("OK MODE set to %s\n", modeToText(controlMode));
    saveConfigToNvs();
    printConfig();
    return;
  }

  if (strcmp(key, "ENTERCNT") == 0) {
    int valueInt = atoi(valueText);
    if (valueInt < 1 || valueInt > 400) {
      Serial.println("ERR ENTERCNT must be in range 1..400");
      return;
    }
    smartEnterCount = (uint16_t)valueInt;
    smartStableCount = 0;
    smartEnterProgressPct = 0.0f;
    Serial.printf("OK ENTERCNT set to %u\n", smartEnterCount);
    saveConfigToNvs();
    printConfig();
    return;
  }

  if (strcmp(key, "EXITCNT") == 0) {
    int valueInt = atoi(valueText);
    if (valueInt < 1 || valueInt > 400) {
      Serial.println("ERR EXITCNT must be in range 1..400");
      return;
    }
    smartExitCount = (uint16_t)valueInt;
    smartDriftCount = 0;
    smartExitProgressPct = 0.0f;
    Serial.printf("OK EXITCNT set to %u\n", smartExitCount);
    saveConfigToNvs();
    printConfig();
    return;
  }

  if (strcmp(key, "RUN") == 0) {
    if (strcmp(valueText, "ON") == 0) {
      controlEnabled = true;
      pid.reset();
      resetSmartState();
      setFanSpeedPercent(fanSpeedPercent);
      Serial.println("OK RUN set to ON");
    } else if (strcmp(valueText, "OFF") == 0) {
      controlEnabled = false;
      setPWM(0);
      ledcWrite(fanPwmChannel, 0);
      fanPwmApplied = 0;
      pid.reset();
      resetSmartState();
      Serial.println("OK RUN set to OFF");
    } else {
      Serial.println("ERR RUN must be ON or OFF");
      return;
    }
    printConfig();
    return;
  }

  if (strcmp(key, "MANPWM") == 0) {
    float valueFloat = atof(valueText);
    if (valueFloat < 0.0f || valueFloat > 255.0f) {
      Serial.println("ERR MANPWM must be in range 0..255");
      return;
    }
    manualPwmTarget = valueFloat;
    Serial.printf("OK MANPWM set to %.2f\n", manualPwmTarget);
    saveConfigToNvs();
    printConfig();
    return;
  }

  if (strcmp(key, "FANINV") == 0) {
    if (strcmp(valueText, "1") == 0 || strcmp(valueText, "ON") == 0 || strcmp(valueText, "TRUE") == 0) {
      fanPwmInverted = true;
    } else if (strcmp(valueText, "0") == 0 || strcmp(valueText, "OFF") == 0 || strcmp(valueText, "FALSE") == 0) {
      fanPwmInverted = false;
    } else {
      Serial.println("ERR FANINV must be 0/1 or OFF/ON");
      return;
    }
    setFanSpeedPercent(fanSpeedPercent);
    Serial.printf("OK FANINV set to %d\n", fanPwmInverted ? 1 : 0);
    saveConfigToNvs();
    printConfig();
    return;
  }

  float value = atof(valueText);
  if (strcmp(key, "KP") == 0) {
    pidKp = value;
    applyPidTunings();
    Serial.printf("OK KP set to %.3f\n", pidKp);
  } else if (strcmp(key, "KI") == 0) {
    pidKi = value;
    applyPidTunings();
    Serial.printf("OK KI set to %.3f\n", pidKi);
  } else if (strcmp(key, "KD") == 0) {
    pidKd = value;
    applyPidTunings();
    Serial.printf("OK KD set to %.3f\n", pidKd);
  } else if (strcmp(key, "BIAS") == 0) {
    if (value < -255.0f || value > 255.0f) {
      Serial.println("ERR BIAS must be in range -255..255");
      return;
    }
    pidBias = value;
    applyPidTunings();
    Serial.printf("OK BIAS set to %.2f\n", pidBias);
  } else if (strcmp(key, "SPBIAS") == 0) {
    if (value < -200.0f || value > 200.0f) {
      Serial.println("ERR SPBIAS must be in range -200..200");
      return;
    }
    setpointBias = value;
    Serial.printf("OK SPBIAS set to %.2f\n", setpointBias);
  } else if (strcmp(key, "SP") == 0) {
    if (value < -20.0f || value > 400.0f) {
      Serial.println("ERR SP must be in range -20..400");
      return;
    }
    setpoint = value;
    Serial.printf("OK SP set to %.2f\n", setpoint);
  } else if (strcmp(key, "ALPHA") == 0) {
    if (value <= 0.0f || value > 1.0f) {
      Serial.println("ERR ALPHA must be in range (0..1]");
      return;
    }
    emaAlpha = value;
    Serial.printf("OK ALPHA set to %.3f\n", emaAlpha);
  } else if (strcmp(key, "FAN") == 0) {
    if (value < 0.0f || value > 100.0f) {
      Serial.println("ERR FAN must be in range 0..100 (percent)");
      return;
    }
    setFanSpeedPercent(value);
    Serial.printf("OK FAN set to %.1f %% (raw %d, 255=off 0=full)\n", fanSpeedPercent, fanPwmRaw);
  } else {
    Serial.println("ERR Unknown key. Use KP, KI, KD, BIAS, SPBIAS, SP, ALPHA, MAXSTEP, ENTERCNT, EXITCNT, FAN, FANINV, MODE, MANPWM, RUN");
    return;
  }

  saveConfigToNvs();
  printConfig();
}

void processSerialCommands() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') {
      continue;
    }
    if (c == '\n') {
      commandBuffer[commandLen] = '\0';
      handleCommand(commandBuffer);
      commandLen = 0;
      continue;
    }
    if (commandLen < (commandBufferSize - 1)) {
      commandBuffer[commandLen++] = c;
    } else {
      commandLen = 0;
      Serial.println("ERR Command too long");
    }
  }
}

void setup()
{
  Serial.begin(115200);
  delay(500);
  preferences.begin("heatctl", false);
  loadConfigFromNvs();

  Wire.begin(i2cSdaPin, i2cSclPin);
  inaOk = ina226.begin();
  if (inaOk) {
    ina226.setMaxCurrentShunt(2.0f, 0.0353f); // 2A max, ~35.3mOhm actual shunt — limit: shunt*max <= 81.92mV
    ina226.setAverage(INA226_1024_SAMPLES);  // average over many samples for PWM accuracy
    ina226.setShuntVoltageConversionTime(INA226_8300_us);
    ina226.setBusVoltageConversionTime(INA226_8300_us);
    Serial.println("INA226 OK");
  } else {
    Serial.println("INA226 NOT FOUND – check wiring");
  }

  ledcSetup(heaterPwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(heaterMosfetPin, heaterPwmChannel);
  ledcSetup(fanPwmChannel, pwmFreq, pwmResolution);
  ledcAttachPin(fanPwmPin, fanPwmChannel);

  setPWM(0);
  setFanSpeedPercent(fanSpeedPercent);
  pid.setBias(pidBias);
  pid.setTunings(pidKp, pidKi, pidKd);
  pid.reset();
  resetSmartState();

  Serial.println("System started");
  Serial.println("Commands: GET, SET KP <v>, SET KI <v>, SET KD <v>, SET BIAS <v>, SET SPBIAS <v>, SET SP <v>, SET ALPHA <v>, SET MAXSTEP <v>, SET ENTERCNT <1..400>, SET EXITCNT <1..400>, SET FAN <0..100>, SET FANINV <0|1>, SET MODE <AUTO|MANUAL|SMART>, SET MANPWM <0..255>, SET RUN <ON|OFF>");
  printConfig();
}

void loop() {
  processSerialCommands();

  uint32_t now = millis();
  if ((now - lastControlMs) < controlPeriodMs) {
    return;
  }
  lastControlMs = now;

  if (inaOk) {
    inaVoltage = supplyVoltage;
    inaCurrent = ina226.getCurrent();
    inaPower   = inaVoltage * inaCurrent;
  }

  float rawTemp = thermocouple.readCelsius();
  float t = readTempFiltered(rawTemp);
  if (isnan(t)) {
    setPWM(0);
    ledcWrite(fanPwmChannel, 0);
    fanPwmApplied = 0;
    pid.reset();
    resetSmartState();
    Serial.println("Temp read error -> heater OFF (NaN/out of range)");
    return;
  }

  if (stuckDetected(t)) {
    setPWM(0);
    ledcWrite(fanPwmChannel, 0);
    fanPwmApplied = 0;
    pid.reset();
    resetSmartState();
    Serial.println("Thermocouple STUCK detected -> heater OFF + PID reset");
    delay(500);
    sameCount = 0;
    return;
  }

  float tSmooth = smoothTemp(t);
  float effectiveSetpoint = setpoint + setpointBias;
  effectiveSetpoint = constrain(effectiveSetpoint, -20.0f, 400.0f);
  float error = effectiveSetpoint - tSmooth;
  float absError = fabsf(error);
  float absErrorMinusAccuracy = max(0.0f, absError - thermocoupleAccuracyC);
  float slope = 0.0f;
  if (!isnan(lastSmoothForSlope)) {
    slope = (tSmooth - lastSmoothForSlope) / ((float)controlPeriodMs / 1000.0f);
  }
  float absSlope = fabsf(slope);
  lastSmoothForSlope = tSmooth;

  int pwm = 0;
  if (!controlEnabled) {
    pwm = 0;
    setPWM(0);
    ledcWrite(fanPwmChannel, 0);
    fanPwmApplied = 0;
    smartHoldActive = false;
    smartStableCount = 0;
    smartDriftCount = 0;
    smartEnterProgressPct = 0.0f;
    smartExitProgressPct = 0.0f;
  } else {
    setFanSpeedPercent(fanSpeedPercent);

    if (controlMode == MODE_MANUAL) {
      pwm = ditheredPwmFromTarget(manualPwmTarget);
      smartHoldActive = false;
      smartEnterProgressPct = 0.0f;
      smartExitProgressPct = 0.0f;
      lastPWM = pwm;
    } else if (controlMode == MODE_AUTO) {
      pwm = (int)pid.calculate(effectiveSetpoint, tSmooth);
      pwm = limitPWMChange(pwm, maxPwmStep);
      smartHoldActive = false;
      smartStableCount = 0;
      smartDriftCount = 0;
      smartEnterProgressPct = 0.0f;
      smartExitProgressPct = 0.0f;
    } else {
      if (!smartHoldActive) {
        pwm = (int)pid.calculate(effectiveSetpoint, tSmooth);
        pwm = limitPWMChange(pwm, maxPwmStep);
        pushSmartPwmSample((float)pwm);

        bool isStable = (absErrorMinusAccuracy <= smartEnterErrC) && (absSlope <= smartEnterSlopeCps);
        if (isStable) {
          if (smartStableCount < smartEnterCount) {
            smartStableCount++;
          }
        } else {
          // Decay instead of hard reset so single noisy samples do not block handover forever.
          if (smartStableCount > 0) {
            smartStableCount--;
          }
        }
        smartEnterProgressPct = 100.0f * ((float)smartStableCount / (float)smartEnterCount);
        if (smartEnterProgressPct > 100.0f) smartEnterProgressPct = 100.0f;
        smartExitProgressPct = 0.0f;

        if (smartStableCount >= smartEnterCount && smartPwmHistCount >= 10) {
          smartHoldPwmTarget = getSmartPwmAverage();
          smartHoldActive = true;
          smartDriftCount = 0;
          pwmDitherAccumulator = 0.0f;
          smartEnterProgressPct = 100.0f;
          Serial.printf("SMART ENTER HOLD | HOLDPWM: %.2f | ERR: %.2f C | SLOPE: %.3f C/s\n",
                        smartHoldPwmTarget, error, slope);
        }
      } else {
        pwm = ditheredPwmFromTarget(smartHoldPwmTarget);
        lastPWM = pwm;
        if (absErrorMinusAccuracy >= smartExitErrC) {
          smartDriftCount++;
        } else {
          smartDriftCount = 0;
        }
        smartEnterProgressPct = 100.0f;
        smartExitProgressPct = 100.0f * ((float)smartDriftCount / (float)smartExitCount);
        if (smartExitProgressPct > 100.0f) smartExitProgressPct = 100.0f;

        if (smartDriftCount >= smartExitCount) {
          smartHoldActive = false;
          smartStableCount = 0;
          smartDriftCount = 0;
          smartPwmHistCount = 0;
          smartPwmHistIndex = 0;
          smartEnterProgressPct = 0.0f;
          smartExitProgressPct = 0.0f;
          pid.reset();
          Serial.printf("SMART EXIT HOLD -> PID | ERR: %.2f C | SLOPE: %.3f C/s\n", error, slope);
        }
      }
    }

    setPWM(pwm);
  }

  // Equilibrium PWM: EMA update only when temperature is near-stable
  if (absSlope < 0.3f && controlEnabled) {
    eqPwm = 0.05f * (float)pwm + 0.95f * eqPwm;
  }

  float pTerm = 0.0f;
  float iTerm = 0.0f;
  float dTerm = 0.0f;
  float pidOut = 0.0f;
  pid.getLastTerms(pTerm, iTerm, dTerm, pidOut);

  const char *stateText = "PID";
  if (controlMode == MODE_MANUAL) {
    stateText = "MANUAL";
  } else if (controlMode == MODE_SMART && smartHoldActive) {
    stateText = "HOLD";
  }

  Serial.printf("Rawtemp %.2f C | Temp: %.2f C | Smooth: %.2f C | PWM: %d | P: %.2f | I: %.2f | D: %.2f | OUT: %.2f | BIAS: %.2f | SPBIAS: %.2f | SP: %.2f | EFFSP: %.2f | FAN: %.1f | FANPWM: %d | MODE: %s | STATE: %s | MANPWM: %.2f | HOLDPWM: %.2f | ENTPROG: %.1f | EXTPROG: %.1f | EABS: %.2f | RUN: %s | FANINV: %d | V: %.3f V | I: %.4f A | W: %.3f W | EQPWM: %.1f\n",
                rawTemp, t, tSmooth, pwm, pTerm, iTerm, dTerm, pidOut, pidBias, setpointBias, setpoint, effectiveSetpoint, fanSpeedPercent, fanPwmApplied,
                modeToText(controlMode), stateText, manualPwmTarget, smartHoldPwmTarget, smartEnterProgressPct, smartExitProgressPct, absError,
                controlEnabled ? "ON" : "OFF", fanPwmInverted ? 1 : 0,
                inaVoltage, inaCurrent, inaPower, eqPwm);
}
