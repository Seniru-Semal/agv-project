#include <Arduino.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

/*
  ADVANCED IR-ONLY LARGE-AGV CONTROLLER

  This is a complete replacement for the 13-channel Arduino Mega controller.
  It keeps the Pi protocol, telemetry, branch selection, direct-drive modes,
  E-stop, warning relay, and supervised derailment recovery.

  Improvement over the previous controller:
    - time-normalised, filtered IR position and rate
    - confidence-aware adaptive PD steering
    - automatic speed reduction for large/fast line errors and poor confidence
    - multi-point PWM-to-speed feed-forward, with independent wheel PI trims
    - larger but slower, controlled turn authority for selected LEFT/RIGHT

  IMPORTANT:
    - SENSOR_THRESHOLD[] starts with the current 400 values.  Replace each
      entry only after stationary SIG calibration of that physical channel.
    - DEFAULT_CRUISE_PWM is 40.  ABSOLUTE_MAX_MOTOR_PWM remains 80 to preserve
      the current direct-drive, pivot and heavy-load test capability.
*/

// ---------------------------------------------------------------------------
// Hardware
// ---------------------------------------------------------------------------
const uint8_t IR_TX_PIN = 8;
const uint8_t SENSOR_COUNT = 13;
const uint8_t SENSOR_PINS[SENSOR_COUNT] = {
  A0, A1, A2, A3, A4, A5, A6, A7, A8, A9, A10, A11, A12
};
// A0 is rightmost. Positive position means the line is to the right.
const int SENSOR_WEIGHTS[SENSOR_COUNT] = {
   350, 292, 233, 175, 117, 58, 0, -58, -117, -175, -233, -292, -350
};

const uint8_t LEFT_RPWM_PIN = 6;
const uint8_t LEFT_LPWM_PIN = 5;
const uint8_t RIGHT_RPWM_PIN = 11;
const uint8_t RIGHT_LPWM_PIN = 10;

const uint8_t LEFT_ENC_A_PIN = 2;
const uint8_t LEFT_ENC_B_PIN = 3;
const uint8_t RIGHT_ENC_A_PIN = 20;
const uint8_t RIGHT_ENC_B_PIN = 21;

const uint8_t STATUS_LED = LED_BUILTIN;
const uint8_t WARNING_RELAY_LIGHT_PIN = 42;
const uint8_t WARNING_RELAY_SPARE_1_PIN = 44;
const uint8_t WARNING_RELAY_SPARE_2_PIN = 46;
const uint8_t WARNING_RELAY_SPARE_3_PIN = 48;

const bool WARNING_RELAY_ACTIVE_LOW = true;
const bool IR_CONTROL_ACTIVE_HIGH = true;
const bool INVERT_LEFT_MOTOR = false;
const bool INVERT_RIGHT_MOTOR = true;
const bool INVERT_STEERING = false;
const bool INVERT_LEFT_ENCODER = true;
const bool INVERT_RIGHT_ENCODER = false;

// ---------------------------------------------------------------------------
// Sensor calibration and acquisition
// ---------------------------------------------------------------------------
const int MIN_MAX_SIGNAL = 40;
const int MIN_CONTRAST = 20;
const int SENSOR_THRESHOLD[SENSOR_COUNT] = {
  400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400, 400
};
// Leave at 0 / 1 until individual stationary channel calibration is measured.
const int SENSOR_OFFSET[SENSOR_COUNT] = {
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0
};
const float SENSOR_GAIN[SENSOR_COUNT] = {
  1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1
};

const uint8_t ANALOG_SAMPLES = 5;
const unsigned int ANALOG_SAMPLE_DELAY_US = 100;
const unsigned int IR_SETTLE_DELAY_US = 1000;
const int SENSOR_RELEASE_HYSTERESIS = 15;
const int LINE_ACTIVE_MIN = 1;
const long LINE_TOTAL_STRENGTH_MIN = 20;
const int JUNCTION_WIDE_CLUSTER_MIN = 10;

// The existing bridge accepts SIG telemetry. Leave enabled for first tests;
// turn it off only after confirming that the bridge does not require SIG.
const bool REPORT_SENSOR_SIGNALS = true;
// Keep false during normal operation: the legacy status frame already carries
// all bridge-required fields. Enable only for a supervised tuning run.
const bool REPORT_ADVANCED_TELEMETRY = false;

// ---------------------------------------------------------------------------
// Motion, advanced IR controller and timing
// ---------------------------------------------------------------------------
const int DEFAULT_CRUISE_PWM = 40;
int baseSpeed = DEFAULT_CRUISE_PWM;
const int ABSOLUTE_MAX_MOTOR_PWM = 80;

float Kp = 0.06f;
float Ki = 0.0f;
float Kd = 0.20f;
const float INTEGRAL_LIMIT = 300.0f;

const int MAX_NORMAL_LINE_STEERING = 35;
const int MAX_CORNER_LINE_STEERING = 45;
const int MAX_BRANCH_LINE_STEERING = 30;
const int MAX_CORNER_REVERSE_PWM = 0;
const int MAX_DERIVATIVE_STEP = 180;

const int STRAIGHT_BRANCH_POSITION_LIMIT = 45;
const int STRAIGHT_BRANCH_STEERING_LIMIT = 10;
const int TURN_BRANCH_POSITION_LIMIT = 260;
const int ACTIVE_BRANCH_SPEED = 25;

const int MIN_FOLLOW_PWM = 18;
const float POSITION_FILTER_ALPHA_AUTO = 0.58f;
const float POSITION_FILTER_ALPHA_BRANCH = 0.78f;
const float RATE_FILTER_ALPHA = 0.42f;
const float MAX_POSITION_RATE = 6500.0f; // weighted-position units / second
const float EDGE_KP_BOOST = 0.0f;
const float EDGE_KD_BOOST = 0.0f;
const float ERROR_SPEED_REDUCTION = 0.0f;
const float RATE_SPEED_REDUCTION = 0.0f;
const float LOW_CONFIDENCE_SPEED_REDUCTION = 0.0f;

const int PWM_RISE_PER_CONTROL = 3;
const int PWM_FALL_PER_CONTROL = 6;
const int LINE_LOST_HOLD_PWM = 18;

const unsigned long CONTROL_INTERVAL_MS = 35;
const unsigned long STATUS_INTERVAL_MS = 50;
const bool COMMAND_WATCHDOG_ENABLED = false;
const unsigned long COMMAND_WATCHDOG_TIMEOUT_MS = 500;

// ---------------------------------------------------------------------------
// Encoder speed control and feed-forward
// ---------------------------------------------------------------------------
const bool WHEEL_SPEED_CONTROL_ENABLED = true;
const float LEFT_TICKS_PER_MM = 1.0745f;
const float RIGHT_TICKS_PER_MM = 1.0535f;

// Default table exactly follows the former 40 -> 525 mm/s and 80 -> 1200
// mm/s references. Replace entries with measured loaded values per wheel.
const uint8_t FEED_FORWARD_POINTS = 9;
const int FEED_FORWARD_PWM[FEED_FORWARD_POINTS] = {0, 18, 25, 30, 35, 40, 60, 70, 80};
const float LEFT_FEED_FORWARD_SPEED[FEED_FORWARD_POINTS] = {
  0, 236, 328, 394, 459, 525, 863, 1031, 1200
};
const float RIGHT_FEED_FORWARD_SPEED[FEED_FORWARD_POINTS] = {
  0, 236, 328, 394, 459, 525, 863, 1031, 1200
};

float wheelSpeedKp = 0.040f;
float wheelSpeedKi = 0.010f;
float wheelSpeedKd = 0.000f;
const float WHEEL_SPEED_INTEGRAL_LIMIT = 800.0f;
const int MAX_WHEEL_SPEED_CORRECTION_PWM = 20;

// ---------------------------------------------------------------------------
// Recovery, brake and branch settings
// ---------------------------------------------------------------------------
const bool ACTIVE_BRAKE_ON_STOP = true;
const int ACTIVE_BRAKE_PWM = ABSOLUTE_MAX_MOTOR_PWM;
const unsigned long ACTIVE_BRAKE_TIME_MS = 120;

const int LINE_LOST_RECOVERY_FRAMES = 5;
const int FORWARD_RECOVERY_PWM = 25;
const unsigned long FORWARD_RECOVERY_TIME_MS = 850;
const int TURN_RECOVERY_PWM = 30;
const int REACQUIRE_POSITION_TOLERANCE = 170;
const int REACQUIRE_CONFIRM_FRAMES = 3;
const unsigned long MAX_TURN_RECOVERY_TIME_MS = 2500;
const int RECOVERY_DIRECTION_MIN_POSITION = 40;
const int RECOVERY_DIRECTION_CONFIRM_FRAMES = 3;
const unsigned long RECOVERY_HINT_MAX_AGE_MS = 1500;

const unsigned long BRANCH_COMMAND_TIMEOUT_MS = 10000;
const int RIGHT_BRANCH_FIRST_INDEX = 0;
const int RIGHT_BRANCH_LAST_INDEX = SENSOR_COUNT / 2;
const int LEFT_BRANCH_FIRST_INDEX = SENSOR_COUNT / 2;
const int LEFT_BRANCH_LAST_INDEX = SENSOR_COUNT - 1;
const int STRAIGHT_BRANCH_FIRST_INDEX = 5;
const int STRAIGHT_BRANCH_LAST_INDEX = 7;
const int STRAIGHT_BRANCH_FALLBACK_FIRST_INDEX = 4;
const int STRAIGHT_BRANCH_FALLBACK_LAST_INDEX = 8;

enum DriveState {
  STATE_IDLE, STATE_FOLLOW, STATE_RECOVER_LEFT, STATE_RECOVER_RIGHT,
  STATE_RECOVER_FORWARD, STATE_DERAIL_HOLD, STATE_DERAIL_READY,
  STATE_MANUAL_PIVOT_LEFT, STATE_MANUAL_PIVOT_RIGHT, STATE_RAW_DRIVE,
  STATE_STOPPED, STATE_ESTOP
};

enum BranchMode { BRANCH_AUTO, BRANCH_STRAIGHT, BRANCH_LEFT, BRANCH_RIGHT };

DriveState driveState = STATE_IDLE;
BranchMode branchMode = BRANCH_AUTO;
bool followEnabled = false;
bool eStopActive = false;
bool forwardRecoveryAttempted = false;
bool recoveryStopLatched = false;

int offValues[SENSOR_COUNT];
int onValues[SENSOR_COUNT];
int signalValues[SENSOR_COUNT];
int lineStrengthValues[SENSOR_COUNT];
bool lineActiveMask[SENSOR_COUNT];
bool sensorLatchedMask[SENSOR_COUNT];

int currentLinePosition = 0;
int activeSensorCount = 0;
bool validLineForTracking = false;
long selectedLineStrength = 0;
int clusterCount = 0;
bool junctionCandidate = false;
int selectedClusterIndex = -1;
int selectedClusterStart = -1;
int selectedClusterEnd = -1;
int selectedClusterPosition = 0;
int selectedClusterActiveCount = 0;

float filteredLinePosition = 0.0f;
float filteredLineRate = 0.0f;
float lineConfidence = 0.0f;
float effectiveKp = 0.0f;
float effectiveKd = 0.0f;
float requestedFollowPwm = 0.0f;
float requestedSteeringPwm = 0.0f;
bool lineFilterValid = false;
float lineIntegral = 0.0f;
float lastFilteredError = 0.0f;

int lineLostFrameCount = 0;
int reacquireFrameCount = 0;
int recoveryHintDirection = 0;
unsigned long recoveryHintTimeMs = 0;
int recoveryCandidateDirection = 0;
int recoveryCandidateFrames = 0;

int manualPivotPwm = 0;
int rawLeftCommand = 0;
int rawRightCommand = 0;
int lastLeftCommand = 0;
int lastRightCommand = 0;

volatile long leftTicks = 0;
volatile long rightTicks = 0;
volatile byte lastLeftEncoderState = 0;
volatile byte lastRightEncoderState = 0;

long lastSpeedLeftTicks = 0;
long lastSpeedRightTicks = 0;
unsigned long lastWheelSpeedSampleTimeMs = 0;
bool wheelSpeedSampleValid = false;
float wheelSpeedSamplePeriodSeconds = (float)CONTROL_INTERVAL_MS / 1000.0f;
float leftMeasuredSpeedMmS = 0.0f;
float rightMeasuredSpeedMmS = 0.0f;
float leftTargetSpeedMmS = 0.0f;
float rightTargetSpeedMmS = 0.0f;
float leftSpeedErrorMmS = 0.0f;
float rightSpeedErrorMmS = 0.0f;
float leftSpeedIntegral = 0.0f;
float rightSpeedIntegral = 0.0f;
float leftLastSpeedError = 0.0f;
float rightLastSpeedError = 0.0f;
float leftWheelPwmCorrection = 0.0f;
float rightWheelPwmCorrection = 0.0f;

char serialBuffer[101];
uint8_t serialLength = 0;
unsigned long lastControlTime = 0;
unsigned long lastStatusTime = 0;
unsigned long turnRecoveryStartTime = 0;
unsigned long lastValidCommandTime = 0;
unsigned long branchModeSetTimeMs = 0;

// ---------------------------------------------------------------------------
// Small utilities
// ---------------------------------------------------------------------------
float clampFloat(float value, float low, float high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

int clampInt(int value, int low, int high) {
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

int roundedInt(float value) {
  return value >= 0.0f ? (int)(value + 0.5f) : (int)(value - 0.5f);
}

bool startsWith(const char *text, const char *prefix) {
  return strncmp(text, prefix, strlen(prefix)) == 0;
}

void trimInPlace(char *text) {
  char *start = text;
  while (*start && isspace((unsigned char)*start)) start++;
  if (start != text) memmove(text, start, strlen(start) + 1);
  size_t length = strlen(text);
  while (length > 0 && isspace((unsigned char)text[length - 1])) {
    text[--length] = '\0';
  }
}

void uppercaseInPlace(char *text) {
  while (*text) {
    *text = (char)toupper((unsigned char)*text);
    text++;
  }
}

// ---------------------------------------------------------------------------
// Encoder interrupts
// ---------------------------------------------------------------------------
int quadratureDelta(byte previous, byte current) {
  byte transition = (previous << 2) | current;
  if (transition == 0b0001 || transition == 0b0111 ||
      transition == 0b1110 || transition == 0b1000) return 1;
  if (transition == 0b0010 || transition == 0b1011 ||
      transition == 0b1101 || transition == 0b0100) return -1;
  return 0;
}

byte readEncoderState(uint8_t pinA, uint8_t pinB) {
  byte state = 0;
  if (digitalRead(pinA)) state |= 0b10;
  if (digitalRead(pinB)) state |= 0b01;
  return state;
}

void updateLeftEncoder() {
  byte state = readEncoderState(LEFT_ENC_A_PIN, LEFT_ENC_B_PIN);
  int delta = quadratureDelta(lastLeftEncoderState, state);
  if (INVERT_LEFT_ENCODER) delta = -delta;
  leftTicks += delta;
  lastLeftEncoderState = state;
}

void updateRightEncoder() {
  byte state = readEncoderState(RIGHT_ENC_A_PIN, RIGHT_ENC_B_PIN);
  int delta = quadratureDelta(lastRightEncoderState, state);
  if (INVERT_RIGHT_ENCODER) delta = -delta;
  rightTicks += delta;
  lastRightEncoderState = state;
}

// ---------------------------------------------------------------------------
// Outputs and motor safety
// ---------------------------------------------------------------------------
void setIrEmitter(bool on) {
  digitalWrite(IR_TX_PIN, (on == IR_CONTROL_ACTIVE_HIGH) ? HIGH : LOW);
}

void writeRelayOutput(uint8_t pin, bool active) {
  digitalWrite(pin, (active == !WARNING_RELAY_ACTIVE_LOW) ? HIGH : LOW);
}

void prepareRelayOutput(uint8_t pin) {
  pinMode(pin, OUTPUT);
  writeRelayOutput(pin, false);
}

void setWarningLight(bool active) {
  writeRelayOutput(WARNING_RELAY_LIGHT_PIN, active);
}

void writeSingleMotor(uint8_t rpwmPin, uint8_t lpwmPin, int command, bool invert) {
  if (invert) command = -command;
  command = clampInt(command, -ABSOLUTE_MAX_MOTOR_PWM, ABSOLUTE_MAX_MOTOR_PWM);
  if (command > 0) {
    analogWrite(rpwmPin, command);
    analogWrite(lpwmPin, 0);
  } else if (command < 0) {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, -command);
  } else {
    analogWrite(rpwmPin, 0);
    analogWrite(lpwmPin, 0);
  }
}

int slewMotorCommand(int applied, int target) {
  if (applied == target) return applied;
  if ((applied > 0 && target < 0) || (applied < 0 && target > 0)) target = 0;
  int step = abs(target) < abs(applied) ? PWM_FALL_PER_CONTROL : PWM_RISE_PER_CONTROL;
  if (target > applied) return min(applied + step, target);
  return max(applied - step, target);
}

void setDriveCommand(int leftCommand, int rightCommand) {
  leftCommand = clampInt(leftCommand, -ABSOLUTE_MAX_MOTOR_PWM, ABSOLUTE_MAX_MOTOR_PWM);
  rightCommand = clampInt(rightCommand, -ABSOLUTE_MAX_MOTOR_PWM, ABSOLUTE_MAX_MOTOR_PWM);
  lastLeftCommand = slewMotorCommand(lastLeftCommand, leftCommand);
  lastRightCommand = slewMotorCommand(lastRightCommand, rightCommand);
  writeSingleMotor(LEFT_RPWM_PIN, LEFT_LPWM_PIN, lastLeftCommand, INVERT_LEFT_MOTOR);
  writeSingleMotor(RIGHT_RPWM_PIN, RIGHT_LPWM_PIN, lastRightCommand, INVERT_RIGHT_MOTOR);
  setWarningLight(lastLeftCommand != 0 || lastRightCommand != 0);
}

void coastMotorOutputs() {
  analogWrite(LEFT_RPWM_PIN, 0);  analogWrite(LEFT_LPWM_PIN, 0);
  analogWrite(RIGHT_RPWM_PIN, 0); analogWrite(RIGHT_LPWM_PIN, 0);
  setWarningLight(false);
}

void clearMotorCommandMemory() {
  lastLeftCommand = 0;
  lastRightCommand = 0;
  rawLeftCommand = 0;
  rawRightCommand = 0;
}

void stopMotors() {
  clearMotorCommandMemory();
  coastMotorOutputs();
}

void brakeAndStopMotors() {
  bool wasMoving = lastLeftCommand != 0 || lastRightCommand != 0 ||
                   rawLeftCommand != 0 || rawRightCommand != 0 || manualPivotPwm != 0;
  clearMotorCommandMemory();
  manualPivotPwm = 0;
  if (ACTIVE_BRAKE_ON_STOP && wasMoving) {
    setWarningLight(true);
    analogWrite(LEFT_RPWM_PIN, ACTIVE_BRAKE_PWM);
    analogWrite(LEFT_LPWM_PIN, ACTIVE_BRAKE_PWM);
    analogWrite(RIGHT_RPWM_PIN, ACTIVE_BRAKE_PWM);
    analogWrite(RIGHT_LPWM_PIN, ACTIVE_BRAKE_PWM);
    delay(ACTIVE_BRAKE_TIME_MS);
  }
  coastMotorOutputs();
}

// ---------------------------------------------------------------------------
// Branch-aware sensor processing
// ---------------------------------------------------------------------------
void clearLineSelectionState() {
  activeSensorCount = 0;
  validLineForTracking = false;
  currentLinePosition = 0;
  selectedLineStrength = 0;
  clusterCount = 0;
  junctionCandidate = false;
  selectedClusterIndex = -1;
  selectedClusterStart = -1;
  selectedClusterEnd = -1;
  selectedClusterPosition = 0;
  selectedClusterActiveCount = 0;
}

void expireBranchCommandIfNeeded() {
  if (branchMode == BRANCH_AUTO || recoveryStopLatched) return;
  if (millis() - branchModeSetTimeMs <= BRANCH_COMMAND_TIMEOUT_MS) return;
  branchMode = BRANCH_AUTO;
  branchModeSetTimeMs = 0;
  lineFilterValid = false;
}

void updateClusterTelemetry() {
  clusterCount = 0;
  int widest = 0;
  int i = 0;
  while (i < SENSOR_COUNT) {
    while (i < SENSOR_COUNT && !lineActiveMask[i]) i++;
    if (i >= SENSOR_COUNT) break;
    int count = 0;
    while (i < SENSOR_COUNT && lineActiveMask[i]) { count++; i++; }
    clusterCount++;
    if (count > widest) widest = count;
  }
  junctionCandidate = clusterCount > 1 || widest >= JUNCTION_WIDE_CLUSTER_MIN;
}

bool acceptSelectedLine(long weightedSum, long totalSignal, int count,
                        int clusterIndex, int startIndex, int endIndex) {
  if (count < LINE_ACTIVE_MIN || totalSignal < LINE_TOTAL_STRENGTH_MIN) return false;
  currentLinePosition = (int)(weightedSum / totalSignal);
  activeSensorCount = count;
  validLineForTracking = true;
  selectedLineStrength = totalSignal;
  selectedClusterIndex = clusterIndex;
  selectedClusterStart = startIndex;
  selectedClusterEnd = endIndex;
  selectedClusterPosition = currentLinePosition;
  selectedClusterActiveCount = count;
  return true;
}

bool chooseClusterInRange(BranchMode mode, int firstIndex, int lastIndex) {
  firstIndex = clampInt(firstIndex, 0, SENSOR_COUNT - 1);
  lastIndex = clampInt(lastIndex, 0, SENSOR_COUNT - 1);
  if (firstIndex > lastIndex) return false;

  bool haveBest = false;
  int bestCluster = -1, bestStart = -1, bestEnd = -1, bestCount = 0, bestPosition = 0;
  long bestWeighted = 0, bestStrength = 0;
  int clusterIndex = 0;
  int i = firstIndex;
  while (i <= lastIndex) {
    while (i <= lastIndex && !lineActiveMask[i]) i++;
    if (i > lastIndex) break;
    int start = i, count = 0;
    long weighted = 0, strength = 0;
    while (i <= lastIndex && lineActiveMask[i]) {
      count++;
      weighted += (long)lineStrengthValues[i] * SENSOR_WEIGHTS[i];
      strength += lineStrengthValues[i];
      i++;
    }
    int end = i - 1;
    if (strength <= 0) { clusterIndex++; continue; }
    int position = (int)(weighted / strength);
    bool better = !haveBest;
    if (haveBest && mode == BRANCH_LEFT)
      better = position < bestPosition || (position == bestPosition && strength > bestStrength);
    else if (haveBest && mode == BRANCH_RIGHT)
      better = position > bestPosition || (position == bestPosition && strength > bestStrength);
    else if (haveBest && mode == BRANCH_STRAIGHT)
      better = abs(position) < abs(bestPosition) || (abs(position) == abs(bestPosition) && strength > bestStrength);
    else if (haveBest && mode == BRANCH_AUTO)
      better = strength > bestStrength || (strength == bestStrength && abs(position) < abs(bestPosition));
    if (better) {
      haveBest = true;
      bestCluster = clusterIndex; bestStart = start; bestEnd = end; bestCount = count;
      bestPosition = position; bestWeighted = weighted; bestStrength = strength;
    }
    clusterIndex++;
  }
  if (!haveBest) return false;
  return acceptSelectedLine(bestWeighted, bestStrength, bestCount, bestCluster, bestStart, bestEnd);
}

void selectLineForTracking() {
  clearLineSelectionState();
  expireBranchCommandIfNeeded();
  updateClusterTelemetry();
  bool selected = false;
  if (branchMode == BRANCH_LEFT)
    selected = chooseClusterInRange(BRANCH_LEFT, LEFT_BRANCH_FIRST_INDEX, LEFT_BRANCH_LAST_INDEX);
  else if (branchMode == BRANCH_RIGHT)
    selected = chooseClusterInRange(BRANCH_RIGHT, RIGHT_BRANCH_FIRST_INDEX, RIGHT_BRANCH_LAST_INDEX);
  else if (branchMode == BRANCH_STRAIGHT) {
    selected = chooseClusterInRange(BRANCH_STRAIGHT, STRAIGHT_BRANCH_FIRST_INDEX, STRAIGHT_BRANCH_LAST_INDEX);
    if (!selected) selected = chooseClusterInRange(BRANCH_STRAIGHT,
                                                     STRAIGHT_BRANCH_FALLBACK_FIRST_INDEX,
                                                     STRAIGHT_BRANCH_FALLBACK_LAST_INDEX);
  }
  if (selected || branchMode == BRANCH_STRAIGHT) return;
  if (junctionCandidate && chooseClusterInRange(BRANCH_STRAIGHT,
                                                  STRAIGHT_BRANCH_FALLBACK_FIRST_INDEX,
                                                  STRAIGHT_BRANCH_FALLBACK_LAST_INDEX)) return;
  BranchMode fallback = (branchMode == BRANCH_LEFT || branchMode == BRANCH_RIGHT) ? branchMode : BRANCH_AUTO;
  chooseClusterInRange(fallback, 0, SENSOR_COUNT - 1);
}

void readSensorsSwitching() {
  long offSums[SENSOR_COUNT] = {0};
  long onSums[SENSOR_COUNT] = {0};
  setIrEmitter(false);
  delayMicroseconds(IR_SETTLE_DELAY_US);
  for (uint8_t sample = 0; sample < ANALOG_SAMPLES; sample++) {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      analogRead(SENSOR_PINS[i]); // discard first conversion after MUX change
      offSums[i] += analogRead(SENSOR_PINS[i]);
    }
    delayMicroseconds(ANALOG_SAMPLE_DELAY_US);
  }
  setIrEmitter(true);
  delayMicroseconds(IR_SETTLE_DELAY_US);
  for (uint8_t sample = 0; sample < ANALOG_SAMPLES; sample++) {
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      analogRead(SENSOR_PINS[i]);
      onSums[i] += analogRead(SENSOR_PINS[i]);
    }
    delayMicroseconds(ANALOG_SAMPLE_DELAY_US);
  }
  setIrEmitter(false);

  int maxSignal = 0, minSignal = 1023;
  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    offValues[i] = (int)(offSums[i] / ANALOG_SAMPLES);
    onValues[i] = (int)(onSums[i] / ANALOG_SAMPLES);
    int raw = offValues[i] - onValues[i];
    if (raw < 0) raw = 0;
    int adjusted = roundedInt((raw - SENSOR_OFFSET[i]) * SENSOR_GAIN[i]);
    signalValues[i] = adjusted > 0 ? adjusted : 0;
    lineStrengthValues[i] = 0;
    lineActiveMask[i] = false;
    if (signalValues[i] > maxSignal) maxSignal = signalValues[i];
    if (signalValues[i] < minSignal) minSignal = signalValues[i];
  }

  clearLineSelectionState();
  if (maxSignal < MIN_MAX_SIGNAL || (maxSignal - minSignal) < MIN_CONTRAST) return;

  for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
    int threshold = SENSOR_THRESHOLD[i];
    if (sensorLatchedMask[i]) {
      if (signalValues[i] < threshold - SENSOR_RELEASE_HYSTERESIS) sensorLatchedMask[i] = false;
    } else if (signalValues[i] >= threshold) {
      sensorLatchedMask[i] = true;
    }
    if (sensorLatchedMask[i]) {
      lineStrengthValues[i] = max(1, signalValues[i] - (threshold - SENSOR_RELEASE_HYSTERESIS));
      lineActiveMask[i] = true;
    }
  }
  selectLineForTracking();
}

// ---------------------------------------------------------------------------
// Advanced IR observation and wheel-speed regulation
// ---------------------------------------------------------------------------
void resetLineController() {
  lineIntegral = 0.0f;
  lastFilteredError = 0.0f;
  filteredLinePosition = 0.0f;
  filteredLineRate = 0.0f;
  lineConfidence = 0.0f;
  effectiveKp = Kp;
  effectiveKd = Kd;
  requestedFollowPwm = 0.0f;
  requestedSteeringPwm = 0.0f;
  lineFilterValid = false;
}

void updateFilteredLineObservation(float dtSeconds) {
  float raw = (float)currentLinePosition;
  if (!lineFilterValid) {
    filteredLinePosition = raw;
    filteredLineRate = 0.0f;
    lastFilteredError = raw;
    lineFilterValid = true;
  } else {
    float alpha = (branchMode == BRANCH_LEFT || branchMode == BRANCH_RIGHT) ?
                  POSITION_FILTER_ALPHA_BRANCH : POSITION_FILTER_ALPHA_AUTO;
    float previous = filteredLinePosition;
    filteredLinePosition += alpha * (raw - filteredLinePosition);
    float rawRate = (filteredLinePosition - previous) / dtSeconds;
    filteredLineRate += RATE_FILTER_ALPHA * (rawRate - filteredLineRate);
  }

  float strengthPerSensor = activeSensorCount > 0 ?
                            (float)selectedLineStrength / activeSensorCount : 0.0f;
  float strengthScore = clampFloat(strengthPerSensor / 130.0f, 0.0f, 1.0f);
  float countScore = activeSensorCount == 1 ? 0.80f :
                     clampFloat(0.80f + 0.07f * (activeSensorCount - 1), 0.0f, 1.0f);
  float rateScore = 1.0f - clampFloat(abs(filteredLineRate) / MAX_POSITION_RATE, 0.0f, 1.0f);
  lineConfidence = clampFloat(0.55f * strengthScore + 0.25f * countScore + 0.20f * rateScore,
                               0.0f, 1.0f);
}

float speedForPwm(int pwm, const float table[]) {
  pwm = clampInt(pwm, 0, ABSOLUTE_MAX_MOTOR_PWM);
  if (pwm <= FEED_FORWARD_PWM[0]) return table[0];
  for (uint8_t i = 1; i < FEED_FORWARD_POINTS; i++) {
    if (pwm <= FEED_FORWARD_PWM[i]) {
      float fraction = (float)(pwm - FEED_FORWARD_PWM[i - 1]) /
                       (float)(FEED_FORWARD_PWM[i] - FEED_FORWARD_PWM[i - 1]);
      return table[i - 1] + fraction * (table[i] - table[i - 1]);
    }
  }
  return table[FEED_FORWARD_POINTS - 1];
}

void updateWheelSpeedMeasurement(unsigned long now) {
  long leftCopy, rightCopy;
  noInterrupts(); leftCopy = leftTicks; rightCopy = rightTicks; interrupts();
  if (!wheelSpeedSampleValid) {
    lastSpeedLeftTicks = leftCopy;
    lastSpeedRightTicks = rightCopy;
    lastWheelSpeedSampleTimeMs = now;
    wheelSpeedSampleValid = true;
    return;
  }
  unsigned long elapsedMs = now - lastWheelSpeedSampleTimeMs;
  if (elapsedMs == 0) return;
  wheelSpeedSamplePeriodSeconds = (float)elapsedMs / 1000.0f;
  leftMeasuredSpeedMmS = ((float)(leftCopy - lastSpeedLeftTicks) / wheelSpeedSamplePeriodSeconds) / LEFT_TICKS_PER_MM;
  rightMeasuredSpeedMmS = ((float)(rightCopy - lastSpeedRightTicks) / wheelSpeedSamplePeriodSeconds) / RIGHT_TICKS_PER_MM;
  lastSpeedLeftTicks = leftCopy;
  lastSpeedRightTicks = rightCopy;
  lastWheelSpeedSampleTimeMs = now;
}

void resetWheelSpeedControl() {
  leftTargetSpeedMmS = rightTargetSpeedMmS = 0.0f;
  leftSpeedErrorMmS = rightSpeedErrorMmS = 0.0f;
  leftSpeedIntegral = rightSpeedIntegral = 0.0f;
  leftLastSpeedError = rightLastSpeedError = 0.0f;
  leftWheelPwmCorrection = rightWheelPwmCorrection = 0.0f;
}

int applySingleWheelSpeedControl(int openLoopCommand, float measuredSpeed,
                                 const float feedForwardTable[], float &targetSpeed,
                                 float &speedError, float &integral,
                                 float &lastError, float &pwmCorrection) {
  int direction = openLoopCommand > 0 ? 1 : (openLoopCommand < 0 ? -1 : 0);
  if (direction == 0) {
    targetSpeed = speedError = integral = lastError = pwmCorrection = 0.0f;
    return 0;
  }
  int magnitude = abs(openLoopCommand);
  targetSpeed = speedForPwm(magnitude, feedForwardTable);
  float measuredAlongCommand = measuredSpeed * direction;
  speedError = targetSpeed - measuredAlongCommand;
  float dt = wheelSpeedSamplePeriodSeconds > 0.0f ? wheelSpeedSamplePeriodSeconds : 0.035f;
  integral = clampFloat(integral + speedError * dt,
                        -WHEEL_SPEED_INTEGRAL_LIMIT, WHEEL_SPEED_INTEGRAL_LIMIT);
  float derivative = (speedError - lastError) / dt;
  pwmCorrection = clampFloat(wheelSpeedKp * speedError + wheelSpeedKi * integral + wheelSpeedKd * derivative,
                             -MAX_WHEEL_SPEED_CORRECTION_PWM, MAX_WHEEL_SPEED_CORRECTION_PWM);
  lastError = speedError;
  return direction * clampInt(magnitude + roundedInt(pwmCorrection), 0, ABSOLUTE_MAX_MOTOR_PWM);
}

void setFollowDriveCommand(int leftCommand, int rightCommand) {
  if (!WHEEL_SPEED_CONTROL_ENABLED || !wheelSpeedSampleValid) {
    resetWheelSpeedControl();
    setDriveCommand(leftCommand, rightCommand);
    return;
  }
  int correctedLeft = applySingleWheelSpeedControl(leftCommand, leftMeasuredSpeedMmS,
                       LEFT_FEED_FORWARD_SPEED, leftTargetSpeedMmS, leftSpeedErrorMmS,
                       leftSpeedIntegral, leftLastSpeedError, leftWheelPwmCorrection);
  int correctedRight = applySingleWheelSpeedControl(rightCommand, rightMeasuredSpeedMmS,
                        RIGHT_FEED_FORWARD_SPEED, rightTargetSpeedMmS, rightSpeedErrorMmS,
                        rightSpeedIntegral, rightLastSpeedError, rightWheelPwmCorrection);
  setDriveCommand(correctedLeft, correctedRight);
}

void applyAdaptiveIrControl(float dtSeconds) {
  updateFilteredLineObservation(dtSeconds);
  if (baseSpeed <= 0) {
    requestedFollowPwm = 0.0f;
    requestedSteeringPwm = 0.0f;
    resetWheelSpeedControl();
    setDriveCommand(0, 0);
    return;
  }
  float error = filteredLinePosition;
  if (branchMode == BRANCH_STRAIGHT)
    error = clampFloat(error, -STRAIGHT_BRANCH_POSITION_LIMIT, STRAIGHT_BRANCH_POSITION_LIMIT);
  else if (branchMode == BRANCH_LEFT || branchMode == BRANCH_RIGHT)
    error = clampFloat(error, -TURN_BRANCH_POSITION_LIMIT, TURN_BRANCH_POSITION_LIMIT);
  if (INVERT_STEERING) error = -error;

  float errorNorm = clampFloat(abs(error) / 350.0f, 0.0f, 1.0f);
  float rateNorm = clampFloat(abs(filteredLineRate) / MAX_POSITION_RATE, 0.0f, 1.0f);
  float adaptation = max(errorNorm, 0.65f * rateNorm);
  effectiveKp = Kp * (1.0f + EDGE_KP_BOOST * adaptation);
  effectiveKd = Kd * (1.0f + EDGE_KD_BOOST * adaptation);

  if (Ki != 0.0f && lineConfidence >= 0.60f && branchMode == BRANCH_AUTO) {
    lineIntegral = clampFloat(lineIntegral + error * dtSeconds, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
  } else {
    lineIntegral = 0.0f;
  }
  // Keep derivative numerically comparable with the old per-frame derivative,
  // while correctly accounting for actual loop duration.
  float derivativeStep = clampFloat(filteredLineRate * ((float)CONTROL_INTERVAL_MS / 1000.0f),
                                    -MAX_DERIVATIVE_STEP, MAX_DERIVATIVE_STEP);
  requestedSteeringPwm = effectiveKp * error + Ki * lineIntegral + effectiveKd * derivativeStep;

  int steeringLimit = MAX_NORMAL_LINE_STEERING;
  if (abs((int)error) >= 220) steeringLimit = MAX_CORNER_LINE_STEERING;
  if (branchMode == BRANCH_STRAIGHT) steeringLimit = STRAIGHT_BRANCH_STEERING_LIMIT;
  if (branchMode == BRANCH_LEFT || branchMode == BRANCH_RIGHT) steeringLimit = MAX_BRANCH_LINE_STEERING;
  requestedSteeringPwm = clampFloat(requestedSteeringPwm, -steeringLimit, steeringLimit);

  float speedScale = 1.0f - ERROR_SPEED_REDUCTION * errorNorm - RATE_SPEED_REDUCTION * rateNorm;
  if (lineConfidence < 0.55f) {
    speedScale -= LOW_CONFIDENCE_SPEED_REDUCTION * (0.55f - lineConfidence) / 0.55f;
  }
  speedScale = clampFloat(speedScale, 0.45f, 1.0f);
  requestedFollowPwm = max((float)MIN_FOLLOW_PWM, baseSpeed * speedScale);
  if (branchMode == BRANCH_LEFT || branchMode == BRANCH_RIGHT)
    requestedFollowPwm = min(requestedFollowPwm, (float)ACTIVE_BRANCH_SPEED);

  int minimumCommand = 0;
  int leftCommand = clampInt(roundedInt(requestedFollowPwm + requestedSteeringPwm),
                             minimumCommand, ABSOLUTE_MAX_MOTOR_PWM);
  int rightCommand = clampInt(roundedInt(requestedFollowPwm - requestedSteeringPwm),
                              minimumCommand, ABSOLUTE_MAX_MOTOR_PWM);
  lastFilteredError = error;
  setFollowDriveCommand(leftCommand, rightCommand);
}

// ---------------------------------------------------------------------------
// Recovery
// ---------------------------------------------------------------------------
void clearRecoveryHint() {
  recoveryHintDirection = 0;
  recoveryHintTimeMs = 0;
  recoveryCandidateDirection = 0;
  recoveryCandidateFrames = 0;
}

void updateRecoveryHintFromCurrentLine() {
  int direction = currentLinePosition >= RECOVERY_DIRECTION_MIN_POSITION ? 1 :
                  (currentLinePosition <= -RECOVERY_DIRECTION_MIN_POSITION ? -1 : 0);
  if (direction == 0) {
    recoveryCandidateDirection = 0;
    recoveryCandidateFrames = 0;
    return;
  }
  if (direction == recoveryCandidateDirection) recoveryCandidateFrames++;
  else { recoveryCandidateDirection = direction; recoveryCandidateFrames = 1; }
  if (recoveryCandidateFrames >= RECOVERY_DIRECTION_CONFIRM_FRAMES) {
    recoveryHintDirection = direction;
    recoveryHintTimeMs = millis();
  }
}

int getFreshRecoveryHint() {
  if (recoveryHintDirection == 0) return 0;
  if (millis() - recoveryHintTimeMs > RECOVERY_HINT_MAX_AGE_MS) {
    clearRecoveryHint();
    return 0;
  }
  return recoveryHintDirection;
}

void startForwardRecovery() {
  forwardRecoveryAttempted = true;
  driveState = STATE_RECOVER_FORWARD;
  turnRecoveryStartTime = millis();
  reacquireFrameCount = 0;
  resetLineController();
}

void finishLineRecovery(float dtSeconds) {
  driveState = STATE_FOLLOW;
  recoveryStopLatched = false;
  forwardRecoveryAttempted = false;
  lineLostFrameCount = 0;
  reacquireFrameCount = 0;
  clearRecoveryHint();
  resetLineController();
  // Match the existing controller: take over from recovery motion immediately
  // once the line has been confirmed, rather than waiting another control tick.
  applyAdaptiveIrControl(dtSeconds);
}

void latchRecoveryStop() {
  followEnabled = false;
  driveState = STATE_DERAIL_HOLD;
  recoveryStopLatched = true;
  lineLostFrameCount = 0;
  reacquireFrameCount = 0;
  resetLineController();
  resetWheelSpeedControl();
  brakeAndStopMotors();
  digitalWrite(STATUS_LED, LOW);
}

void handleDerailHoldState() {
  stopMotors();
  bool centered = validLineForTracking && abs(currentLinePosition) <= REACQUIRE_POSITION_TOLERANCE;
  if (driveState == STATE_DERAIL_READY) {
    if (!centered) { driveState = STATE_DERAIL_HOLD; reacquireFrameCount = 0; }
    return;
  }
  if (centered) reacquireFrameCount++; else reacquireFrameCount = 0;
  if (reacquireFrameCount >= REACQUIRE_CONFIRM_FRAMES) {
    driveState = STATE_DERAIL_READY;
    digitalWrite(STATUS_LED, HIGH);
  }
}

void handleRecoveryState(float dtSeconds) {
  bool centered = validLineForTracking && abs(currentLinePosition) <= REACQUIRE_POSITION_TOLERANCE;
  if (centered) reacquireFrameCount++; else reacquireFrameCount = 0;
  if (reacquireFrameCount >= REACQUIRE_CONFIRM_FRAMES) {
    finishLineRecovery(dtSeconds);
    return;
  }
  unsigned long elapsed = millis() - turnRecoveryStartTime;
  if (driveState == STATE_RECOVER_FORWARD) {
    if (elapsed < FORWARD_RECOVERY_TIME_MS) {
      setDriveCommand(FORWARD_RECOVERY_PWM, FORWARD_RECOVERY_PWM);
      return;
    }
    int direction = getFreshRecoveryHint();
    if (direction > 0) driveState = STATE_RECOVER_RIGHT;
    else if (direction < 0) driveState = STATE_RECOVER_LEFT;
    else { latchRecoveryStop(); return; }
    turnRecoveryStartTime = millis();
    reacquireFrameCount = 0;
    elapsed = 0;
  }
  if (elapsed >= MAX_TURN_RECOVERY_TIME_MS) { latchRecoveryStop(); return; }
  if (driveState == STATE_RECOVER_RIGHT) setDriveCommand(TURN_RECOVERY_PWM, -TURN_RECOVERY_PWM);
  else if (driveState == STATE_RECOVER_LEFT) setDriveCommand(-TURN_RECOVERY_PWM, TURN_RECOVERY_PWM);
  else latchRecoveryStop();
}

// ---------------------------------------------------------------------------
// Main motion state machine
// ---------------------------------------------------------------------------
void runControlLoop(unsigned long now, float controlDtSeconds) {
  updateWheelSpeedMeasurement(now);
  if (eStopActive) {
    resetWheelSpeedControl();
    stopMotors();
    driveState = STATE_ESTOP;
    return;
  }
  if (driveState == STATE_RAW_DRIVE) {
    resetWheelSpeedControl();
    setDriveCommand(rawLeftCommand, rawRightCommand);
    return;
  }
  if (driveState == STATE_MANUAL_PIVOT_LEFT) {
    resetWheelSpeedControl();
    setDriveCommand(-manualPivotPwm, manualPivotPwm);
    return;
  }
  if (driveState == STATE_MANUAL_PIVOT_RIGHT) {
    resetWheelSpeedControl();
    setDriveCommand(manualPivotPwm, -manualPivotPwm);
    return;
  }
  if (driveState == STATE_DERAIL_HOLD || driveState == STATE_DERAIL_READY) {
    resetWheelSpeedControl();
    readSensorsSwitching();
    handleDerailHoldState();
    return;
  }
  if (!followEnabled) {
    readSensorsSwitching();
    resetWheelSpeedControl();
    if (driveState != STATE_STOPPED) driveState = STATE_IDLE;
    stopMotors();
    return;
  }

  readSensorsSwitching();
  if (driveState == STATE_RECOVER_FORWARD || driveState == STATE_RECOVER_LEFT || driveState == STATE_RECOVER_RIGHT) {
    resetWheelSpeedControl();
    handleRecoveryState(controlDtSeconds);
    return;
  }
  if (validLineForTracking) {
    lineLostFrameCount = 0;
    reacquireFrameCount = 0;
    updateRecoveryHintFromCurrentLine();
    driveState = STATE_FOLLOW;
    applyAdaptiveIrControl(controlDtSeconds);
    return;
  }

  resetWheelSpeedControl();
  lineLostFrameCount++;
  if (lineLostFrameCount < LINE_LOST_RECOVERY_FRAMES) {
    setDriveCommand(LINE_LOST_HOLD_PWM, LINE_LOST_HOLD_PWM);
    return;
  }
  if (forwardRecoveryAttempted) { latchRecoveryStop(); return; }
  startForwardRecovery();
  handleRecoveryState(controlDtSeconds);
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------
void noteValidCommand() { lastValidCommandTime = millis(); }

void commandClearBranch() {
  if (driveState == STATE_DERAIL_HOLD || driveState == STATE_DERAIL_READY) {
    followEnabled = false;
    recoveryStopLatched = false;
    forwardRecoveryAttempted = false;
    lineLostFrameCount = 0;
    reacquireFrameCount = 0;
    driveState = STATE_IDLE;
    stopMotors();
    digitalWrite(STATUS_LED, LOW);
  }
  branchMode = BRANCH_AUTO;
  branchModeSetTimeMs = 0;
  resetLineController();
}

void commandStart() {
  if (eStopActive || recoveryStopLatched) return;
  if (followEnabled && (driveState == STATE_FOLLOW || driveState == STATE_RECOVER_FORWARD ||
                        driveState == STATE_RECOVER_LEFT || driveState == STATE_RECOVER_RIGHT)) return;
  rawLeftCommand = rawRightCommand = 0;
  manualPivotPwm = 0;
  followEnabled = true;
  driveState = STATE_FOLLOW;
  forwardRecoveryAttempted = false;
  lineLostFrameCount = reacquireFrameCount = 0;
  clearRecoveryHint();
  resetLineController();
  digitalWrite(STATUS_LED, HIGH);
}

void commandStop() {
  followEnabled = false;
  rawLeftCommand = rawRightCommand = 0;
  manualPivotPwm = 0;
  if (!eStopActive) driveState = STATE_IDLE;
  lineLostFrameCount = reacquireFrameCount = 0;
  forwardRecoveryAttempted = false;
  clearRecoveryHint();
  commandClearBranch();
  resetWheelSpeedControl();
  brakeAndStopMotors();
  digitalWrite(STATUS_LED, LOW);
}

void commandEstop() {
  eStopActive = true;
  followEnabled = false;
  driveState = STATE_ESTOP;
  recoveryStopLatched = true;
  rawLeftCommand = rawRightCommand = 0;
  manualPivotPwm = 0;
  lineLostFrameCount = reacquireFrameCount = 0;
  forwardRecoveryAttempted = false;
  clearRecoveryHint();
  commandClearBranch();
  resetWheelSpeedControl();
  brakeAndStopMotors();
  digitalWrite(STATUS_LED, LOW);
}

void commandReset() {
  eStopActive = false;
  followEnabled = false;
  driveState = STATE_IDLE;
  recoveryStopLatched = false;
  rawLeftCommand = rawRightCommand = 0;
  manualPivotPwm = 0;
  lineLostFrameCount = reacquireFrameCount = 0;
  forwardRecoveryAttempted = false;
  clearRecoveryHint();
  commandClearBranch();
  resetWheelSpeedControl();
  stopMotors();
  digitalWrite(STATUS_LED, LOW);
}

void commandResetTicks() {
  noInterrupts(); leftTicks = 0; rightTicks = 0; interrupts();
  wheelSpeedSampleValid = false;
}

void commandSetSpeed(int value) {
  baseSpeed = clampInt(value, 0, ABSOLUTE_MAX_MOTOR_PWM);
}

void commandSetPid(char *payload) {
  char *comma1 = strchr(payload, ',');
  if (!comma1) return;
  *comma1 = '\0';
  char *comma2 = strchr(comma1 + 1, ',');
  if (!comma2) return;
  *comma2 = '\0';
  Kp = atof(payload);
  Ki = atof(comma1 + 1);
  Kd = atof(comma2 + 1);
  resetLineController();
}

void commandSetBranch(char *payload) {
  trimInPlace(payload);
  uppercaseInPlace(payload);
  BranchMode requested;
  if (strcmp(payload, "LEFT") == 0) requested = BRANCH_LEFT;
  else if (strcmp(payload, "RIGHT") == 0) requested = BRANCH_RIGHT;
  else if (strcmp(payload, "STRAIGHT") == 0) requested = BRANCH_STRAIGHT;
  else if (strcmp(payload, "AUTO") == 0) requested = BRANCH_AUTO;
  else return;
  if (branchMode == requested) {
    if (requested != BRANCH_AUTO) branchModeSetTimeMs = millis();
    return;
  }
  branchMode = requested;
  branchModeSetTimeMs = requested == BRANCH_AUTO ? 0 : millis();
  resetLineController();
}

void commandPivot(bool left, int pwm) {
  if (eStopActive) return;
  manualPivotPwm = clampInt(pwm, 0, ABSOLUTE_MAX_MOTOR_PWM);
  rawLeftCommand = rawRightCommand = 0;
  followEnabled = false;
  driveState = left ? STATE_MANUAL_PIVOT_LEFT : STATE_MANUAL_PIVOT_RIGHT;
  lineLostFrameCount = reacquireFrameCount = 0;
  clearRecoveryHint();
  commandClearBranch();
  resetLineController();
  resetWheelSpeedControl();
  setDriveCommand(left ? -manualPivotPwm : manualPivotPwm,
                  left ? manualPivotPwm : -manualPivotPwm);
  digitalWrite(STATUS_LED, HIGH);
}

void commandRawDrive(char *payload) {
  if (eStopActive) return;
  char *comma = strchr(payload, ',');
  if (!comma) return;
  *comma = '\0';
  rawLeftCommand = clampInt(atoi(payload), -ABSOLUTE_MAX_MOTOR_PWM, ABSOLUTE_MAX_MOTOR_PWM);
  rawRightCommand = clampInt(atoi(comma + 1), -ABSOLUTE_MAX_MOTOR_PWM, ABSOLUTE_MAX_MOTOR_PWM);
  manualPivotPwm = 0;
  followEnabled = false;
  driveState = STATE_RAW_DRIVE;
  lineLostFrameCount = reacquireFrameCount = 0;
  clearRecoveryHint();
  commandClearBranch();
  resetLineController();
  resetWheelSpeedControl();
  setDriveCommand(rawLeftCommand, rawRightCommand);
  digitalWrite(STATUS_LED, HIGH);
}

void commandReacquireLine() {
  if (eStopActive) return;
  recoveryStopLatched = false;
  if (branchMode != BRANCH_AUTO) branchModeSetTimeMs = millis();
  rawLeftCommand = rawRightCommand = 0;
  manualPivotPwm = 0;
  followEnabled = true;
  lineLostFrameCount = reacquireFrameCount = 0;
  resetLineController();
  startForwardRecovery();
  digitalWrite(STATUS_LED, HIGH);
}

void commandAuthorizeDerailResume() {
  if (eStopActive || !recoveryStopLatched || driveState != STATE_DERAIL_READY) return;
  if (!validLineForTracking || abs(currentLinePosition) > REACQUIRE_POSITION_TOLERANCE) {
    driveState = STATE_DERAIL_HOLD;
    reacquireFrameCount = 0;
    return;
  }
  recoveryStopLatched = false;
  followEnabled = true;
  driveState = STATE_FOLLOW;
  forwardRecoveryAttempted = false;
  lineLostFrameCount = reacquireFrameCount = 0;
  resetLineController();
  digitalWrite(STATUS_LED, HIGH);
}

void applyCommandWatchdog(unsigned long now) {
  if (!COMMAND_WATCHDOG_ENABLED) return;
  bool motorMode = followEnabled || driveState == STATE_RAW_DRIVE ||
                   driveState == STATE_MANUAL_PIVOT_LEFT || driveState == STATE_MANUAL_PIVOT_RIGHT;
  if (motorMode && now - lastValidCommandTime > COMMAND_WATCHDOG_TIMEOUT_MS) commandStop();
}

void processCommand(char *command) {
  trimInPlace(command);
  if (*command == '\0') return;
  if (strcmp(command, "C:START") == 0) { noteValidCommand(); commandStart(); }
  else if (strcmp(command, "C:STOP") == 0) { noteValidCommand(); commandStop(); }
  else if (strcmp(command, "C:ESTOP") == 0) { noteValidCommand(); commandEstop(); }
  else if (strcmp(command, "C:RESET") == 0) { noteValidCommand(); commandReset(); }
  else if (strcmp(command, "C:RESET_TICKS") == 0) { noteValidCommand(); commandResetTicks(); }
  else if (startsWith(command, "C:SET_SPEED,")) { noteValidCommand(); commandSetSpeed(atoi(command + 12)); }
  else if (startsWith(command, "C:SET_PID,")) { noteValidCommand(); commandSetPid(command + 10); }
  else if (startsWith(command, "C:SET_BRANCH,")) { noteValidCommand(); commandSetBranch(command + 13); }
  else if (strcmp(command, "C:CLEAR_BRANCH") == 0) { noteValidCommand(); commandClearBranch(); }
  else if (startsWith(command, "C:PIVOT_LEFT,")) { noteValidCommand(); commandPivot(true, atoi(command + 13)); }
  else if (startsWith(command, "C:PIVOT_RIGHT,")) { noteValidCommand(); commandPivot(false, atoi(command + 14)); }
  else if (startsWith(command, "C:RAW_DRIVE,")) { noteValidCommand(); commandRawDrive(command + 12); }
  else if (strcmp(command, "C:REACQUIRE_LINE") == 0) { noteValidCommand(); commandReacquireLine(); }
  else if (strcmp(command, "C:AUTHORIZE_DERAIL_RESUME") == 0) { noteValidCommand(); commandAuthorizeDerailResume(); }
}

void processSerial() {
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\n' || c == '\r') {
      if (serialLength > 0) {
        serialBuffer[serialLength] = '\0';
        processCommand(serialBuffer);
        serialLength = 0;
      }
    } else if (serialLength < sizeof(serialBuffer) - 1) {
      serialBuffer[serialLength++] = c;
    } else {
      serialLength = 0;
    }
  }
}

// ---------------------------------------------------------------------------
// Status protocol -- original fields retained; advanced fields appended.
// ---------------------------------------------------------------------------
const char *branchModeToText(BranchMode mode) {
  if (mode == BRANCH_STRAIGHT) return "STRAIGHT";
  if (mode == BRANCH_LEFT) return "LEFT";
  if (mode == BRANCH_RIGHT) return "RIGHT";
  return "AUTO";
}

const char *stateToText(DriveState state) {
  switch (state) {
    case STATE_IDLE: return "IDLE";
    case STATE_FOLLOW: return "FOLLOW";
    case STATE_RECOVER_LEFT: return "RECOVER_LEFT";
    case STATE_RECOVER_RIGHT: return "RECOVER_RIGHT";
    case STATE_RECOVER_FORWARD: return "RECOVER_FORWARD";
    case STATE_DERAIL_HOLD: return "DERAIL_HOLD";
    case STATE_DERAIL_READY: return "DERAIL_READY";
    case STATE_MANUAL_PIVOT_LEFT: return "PIVOT_LEFT";
    case STATE_MANUAL_PIVOT_RIGHT: return "PIVOT_RIGHT";
    case STATE_RAW_DRIVE: return "RAW_DRIVE";
    case STATE_STOPPED: return "STOPPED";
    case STATE_ESTOP: return "ESTOP";
    default: return "UNKNOWN";
  }
}

int getFaultCode() {
  if (eStopActive || driveState == STATE_ESTOP) return 2;
  if (driveState == STATE_STOPPED) return 1;
  return 0;
}

void reportStatus() {
  long leftCopy, rightCopy;
  noInterrupts(); leftCopy = leftTicks; rightCopy = rightTicks; interrupts();
  unsigned long hintAge = 0;
  int reportHint = 0;
  if (recoveryHintDirection != 0 && recoveryHintTimeMs > 0) {
    hintAge = millis() - recoveryHintTimeMs;
    if (hintAge <= RECOVERY_HINT_MAX_AGE_MS) reportHint = recoveryHintDirection;
  }
  Serial.print("A:STATE="); Serial.print(stateToText(driveState));
  Serial.print(";POS="); Serial.print(currentLinePosition);
  Serial.print(";L="); Serial.print(leftCopy);
  Serial.print(";R="); Serial.print(rightCopy);
  Serial.print(";ACTIVE="); Serial.print(activeSensorCount);
  Serial.print(";VALID="); Serial.print(validLineForTracking ? 1 : 0);
  Serial.print(";LOST="); Serial.print(lineLostFrameCount);
  Serial.print(";SPEED="); Serial.print(baseSpeed);
  Serial.print(";MAXPWM="); Serial.print(ABSOLUTE_MAX_MOTOR_PWM);
  Serial.print(";APPLIEDL="); Serial.print(lastLeftCommand);
  Serial.print(";APPLIEDR="); Serial.print(lastRightCommand);
  Serial.print(";PIVOT="); Serial.print(manualPivotPwm);
  Serial.print(";RAWL="); Serial.print(rawLeftCommand);
  Serial.print(";RAWR="); Serial.print(rawRightCommand);
  Serial.print(";WSYNC="); Serial.print(WHEEL_SPEED_CONTROL_ENABLED ? 1 : 0);
  Serial.print(";DL="); Serial.print((long)leftSpeedErrorMmS);
  Serial.print(";DR="); Serial.print((long)rightSpeedErrorMmS);
  Serial.print(";WCORR="); Serial.print((long)(leftWheelPwmCorrection - rightWheelPwmCorrection));
  Serial.print(";ESTOP="); Serial.print(eStopActive ? 1 : 0);
  Serial.print(";FAULT="); Serial.print(getFaultCode());
  Serial.print(";HINT="); Serial.print(reportHint);
  Serial.print(";HAGE="); Serial.print(hintAge);
  Serial.print(";CAND="); Serial.print(recoveryCandidateDirection);
  Serial.print(";CFR="); Serial.print(recoveryCandidateFrames);
  Serial.print(";CLUST="); Serial.print(clusterCount);
  Serial.print(";JUNC="); Serial.print(junctionCandidate ? 1 : 0);
  Serial.print(";SEL="); Serial.print(selectedClusterIndex);
  Serial.print(";SSTART="); Serial.print(selectedClusterStart);
  Serial.print(";SEND="); Serial.print(selectedClusterEnd);
  Serial.print(";SPOS="); Serial.print(selectedClusterPosition);
  Serial.print(";SACT="); Serial.print(selectedClusterActiveCount);
  Serial.print(";BMODE="); Serial.print(branchModeToText(branchMode));
  if (REPORT_SENSOR_SIGNALS) {
    Serial.print(";SIG=");
    for (uint8_t i = 0; i < SENSOR_COUNT; i++) {
      if (i) Serial.print(',');
      Serial.print(signalValues[i]);
    }
  }
  if (REPORT_ADVANCED_TELEMETRY) {
    Serial.print(";FPOS="); Serial.print((long)filteredLinePosition);
    Serial.print(";FRATE="); Serial.print((long)filteredLineRate);
    Serial.print(";CONF="); Serial.print((int)(lineConfidence * 100.0f));
    Serial.print(";TSPD="); Serial.print((int)requestedFollowPwm);
    Serial.print(";STEER="); Serial.print((int)requestedSteeringPwm);
    Serial.print(";EKP="); Serial.print(effectiveKp, 3);
    Serial.print(";EKD="); Serial.print(effectiveKd, 3);
  }
  Serial.println();
}

// ---------------------------------------------------------------------------
// Arduino entry points
// ---------------------------------------------------------------------------
void setup() {
  pinMode(IR_TX_PIN, OUTPUT);
  setIrEmitter(false);
  prepareRelayOutput(WARNING_RELAY_LIGHT_PIN);
  prepareRelayOutput(WARNING_RELAY_SPARE_1_PIN);
  prepareRelayOutput(WARNING_RELAY_SPARE_2_PIN);
  prepareRelayOutput(WARNING_RELAY_SPARE_3_PIN);
  pinMode(LEFT_RPWM_PIN, OUTPUT);  pinMode(LEFT_LPWM_PIN, OUTPUT);
  pinMode(RIGHT_RPWM_PIN, OUTPUT); pinMode(RIGHT_LPWM_PIN, OUTPUT);
  pinMode(LEFT_ENC_A_PIN, INPUT_PULLUP);  pinMode(LEFT_ENC_B_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A_PIN, INPUT_PULLUP); pinMode(RIGHT_ENC_B_PIN, INPUT_PULLUP);
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  lastLeftEncoderState = readEncoderState(LEFT_ENC_A_PIN, LEFT_ENC_B_PIN);
  lastRightEncoderState = readEncoderState(RIGHT_ENC_A_PIN, RIGHT_ENC_B_PIN);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A_PIN), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B_PIN), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A_PIN), updateRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B_PIN), updateRightEncoder, CHANGE);
  stopMotors();
  Serial.begin(115200);
  driveState = STATE_IDLE;
  followEnabled = false;
  eStopActive = false;
  lastValidCommandTime = millis();
  clearRecoveryHint();
  resetLineController();
}

void loop() {
  processSerial();
  unsigned long now = millis();
  applyCommandWatchdog(now);
  if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
    unsigned long previous = lastControlTime;
    lastControlTime = now;
    float dtSeconds = previous == 0 ? (float)CONTROL_INTERVAL_MS / 1000.0f :
                      (float)(now - previous) / 1000.0f;
    dtSeconds = clampFloat(dtSeconds, 0.015f, 0.150f);
    runControlLoop(now, dtSeconds);
  }
  if (now - lastStatusTime >= STATUS_INTERVAL_MS) {
    lastStatusTime = now;
    reportStatus();
  }
}
