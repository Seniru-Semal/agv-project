/*
  AGV1 LARGE-CHASSIS MIGRATION - 13-SENSOR WHITE LINE FOLLOWER

  Refinements preserve the Pi/RFID-controlled junction architecture.

  Hardware:
  - Arduino Mega
  - 13-channel switching IR array on A0-A12
  - IR emitter control on D8
  - Left IBT-2/BTS7960 driver:  RPWM D6,  LPWM D5
  - Right IBT-2/BTS7960 driver: RPWM D11, LPWM D10
  - Quadrature encoders:
      Left  A/B = D2 / D3
      Right A/B = D18 / D19

  Behavior:
  - White line on darker floor
  - Common-emitter IR receiver circuit
  - SIG = IR_OFF - IR_ON
  - White line gives higher SIG value than floor
*/

// ==================================================
// Pin configuration
// ==================================================

const int IR_TX_PIN = 8;

const int SENSOR_COUNT = 13;
const int SENSOR_PINS[SENSOR_COUNT] = {
  A0, A1, A2, A3, A4, A5, A6,
  A7, A8, A9, A10, A11, A12
};

// A0 = rightmost, A12 = leftmost
// Positive POS = line toward right side
// Negative POS = line toward left side
const int SENSOR_WEIGHTS[SENSOR_COUNT] = {
   350,  292,  233,  175,  117,   58,    0,
   -58, -117, -175, -233, -292, -350
};

// Left motor driver
const int LEFT_RPWM_PIN = 6;
const int LEFT_LPWM_PIN = 5;

// Right motor driver
const int RIGHT_RPWM_PIN = 11;
const int RIGHT_LPWM_PIN = 10;

// Encoders
const int LEFT_ENC_A_PIN = 2;
const int LEFT_ENC_B_PIN = 3;
const int RIGHT_ENC_A_PIN = 18;
const int RIGHT_ENC_B_PIN = 19;

const int STATUS_LED = LED_BUILTIN;

// Active-low relay module.
// Input 1 on D42 drives the warning light.
// Inputs 2-4 are defined now so they can be used later without repinning.
const int WARNING_RELAY_LIGHT_PIN = 42;
const int WARNING_RELAY_SPARE_1_PIN = 44;
const int WARNING_RELAY_SPARE_2_PIN = 46;
const int WARNING_RELAY_SPARE_3_PIN = 48;

const bool WARNING_RELAY_ACTIVE_LOW = true;

// ==================================================
// Hardware calibration
// ==================================================

const bool IR_CONTROL_ACTIVE_HIGH = true;

const bool INVERT_LEFT_MOTOR = false;
const bool INVERT_RIGHT_MOTOR = true;

const bool INVERT_STEERING = false;

const bool INVERT_LEFT_ENCODER = true;
const bool INVERT_RIGHT_ENCODER = false;

// ==================================================
// IR detection settings
// ==================================================

const int MIN_MAX_SIGNAL = 40;
const int MIN_CONTRAST = 20;

// White-line detection:
// signalValues[i] >= SENSOR_SIGNAL_THRESHOLD[i]
const int SENSOR_SIGNAL_THRESHOLD[SENSOR_COUNT] = {
  /* A0  rightmost */ 400,
  /* A1            */ 400,
  /* A2            */ 400,
  /* A3            */ 400,
  /* A4            */ 400,
  /* A5            */ 400,
  /* A6  centre    */ 400,
  /* A7            */ 400,
  /* A8            */ 400,
  /* A9            */ 400,
  /* A10           */ 400,
  /* A11           */ 400,
  /* A12 leftmost  */ 400
};

const int LINE_ACTIVE_MIN = 1;
const int LINE_TOTAL_STRENGTH_MIN = 20;

// Keep full 13-channel SIG telemetry off during normal driving.  Turn this on
// only for stationary calibration or a short supervised diagnostic run.
const bool REPORT_SENSOR_SIGNALS = true;  // Temporary IR diagnostic telemetry

const int ANALOG_SAMPLES = 5;
const int ANALOG_SAMPLE_DELAY_US = 100;
const int IR_SETTLE_DELAY_US = 1000;

// A sensor turns on at its calibrated threshold and stays on until its signal
// falls this far below the threshold.  This suppresses threshold chatter
// without changing the calibrated turn-on point.
const int SENSOR_RELEASE_HYSTERESIS = 15;

// ==================================================
// PID settings
// ==================================================

float Kp = 0.115;
float Ki = 0.00;
float Kd = 1.0;

int baseSpeed = 40;

// Hard safety ceiling for every normal, pivot, raw-drive, recovery and brake
// command. Do not raise this value on the large chassis.
const int ABSOLUTE_MAX_MOTOR_PWM = 80;

const int MAX_NORMAL_LINE_STEERING = 35;
const int MAX_CORNER_LINE_STEERING = 45;
const int MAX_CORNER_REVERSE_PWM = 0;
const int MAX_DERIVATIVE_STEP = 180;

const int PWM_RISE_PER_CONTROL = 3;
const int PWM_FALL_PER_CONTROL = 6;

// Never keep the previous strong steering command while the line is invalid.
const int LINE_LOST_HOLD_PWM = 18;

const float INTEGRAL_LIMIT = 300.0;

// ==================================================
// Follow-only encoder speed control
// ==================================================
// Measured on the large AGV over 2,000 mm:
// left = 2,149 ticks, right = 2,107 ticks.
const bool WHEEL_SPEED_CONTROL_ENABLED = true;
const float LEFT_TICKS_PER_MM = 1.0745f;
const float RIGHT_TICKS_PER_MM = 1.0535f;

// Measured loaded straight-drive reference points:
// PWM 40 for 5 s -> 525 mm/s vehicle average.
// PWM 80 for 4 s -> 1,200 mm/s vehicle average.
const int SPEED_REFERENCE_PWM_LOW = 40;
const int SPEED_REFERENCE_PWM_HIGH = 80;
const float FOLLOW_SPEED_MM_S_AT_PWM_40 = 525.0f;
const float FOLLOW_SPEED_MM_S_AT_PWM_80 = 1200.0f;

// These are wheel-speed PID gains in PWM per (mm/s).  They apply only while
// STATE_FOLLOW owns the motors; raw drive, pivots and recovery remain direct.
float wheelSpeedKp = 0.040f;
float wheelSpeedKi = 0.010f;
float wheelSpeedKd = 0.000f;
const float WHEEL_SPEED_INTEGRAL_LIMIT = 800.0f;
const int MAX_WHEEL_SPEED_CORRECTION_PWM = 20;

// ==================================================
// Active braking
// ==================================================

const bool ACTIVE_BRAKE_ON_STOP = true;
const int ACTIVE_BRAKE_PWM = ABSOLUTE_MAX_MOTOR_PWM;
const unsigned long ACTIVE_BRAKE_TIME_MS = 120;

// ==================================================
// Line recovery settings
// ==================================================

const int LINE_LOST_RECOVERY_FRAMES = 5;

const int FORWARD_RECOVERY_PWM = 25;
const unsigned long FORWARD_RECOVERY_TIME_MS = 850;

const int TURN_RECOVERY_PWM = 30;

const int CORNER_DETECT_POSITION = 220;

const int REACQUIRE_POSITION_TOLERANCE = 170;
const int REACQUIRE_CONFIRM_FRAMES = 3;
const unsigned long MAX_TURN_RECOVERY_TIME_MS = 2500;

const int RECOVERY_DIRECTION_MIN_POSITION = 40;
const int RECOVERY_DIRECTION_CONFIRM_FRAMES = 3;
const unsigned long RECOVERY_HINT_MAX_AGE_MS = 1500;

// ==================================================
// Junction cluster telemetry settings
// ==================================================

const int JUNCTION_WIDE_CLUSTER_MIN = 10;

// ==================================================
// Junction branch selection settings
// ==================================================

const unsigned long BRANCH_COMMAND_TIMEOUT_MS = 10000;

const int RIGHT_BRANCH_FIRST_INDEX = 0;
const int RIGHT_BRANCH_LAST_INDEX = SENSOR_COUNT / 2;

const int LEFT_BRANCH_FIRST_INDEX = SENSOR_COUNT / 2;
const int LEFT_BRANCH_LAST_INDEX = SENSOR_COUNT - 1;

const int STRAIGHT_BRANCH_FIRST_INDEX = 5;
const int STRAIGHT_BRANCH_LAST_INDEX = 7;

const int STRAIGHT_BRANCH_FALLBACK_FIRST_INDEX = 4;
const int STRAIGHT_BRANCH_FALLBACK_LAST_INDEX = 8;

const int STRAIGHT_BRANCH_POSITION_LIMIT = 45;
const int TURN_BRANCH_POSITION_LIMIT = 140;

const int STRAIGHT_BRANCH_STEERING_LIMIT = 10;
const int TURN_BRANCH_STEERING_LIMIT = 18;

// The Pi/RFID still chooses LEFT or RIGHT.  This only gives a selected branch
// enough turning authority without ever raising the physical PWM ceiling.
const int ACTIVE_BRANCH_SPEED = 25;

// ==================================================
// Timing
// ==================================================

// Five averaged OFF reads plus five averaged ON reads cannot complete safely
// in 20 ms on an Arduino Mega.  A fixed 35 ms period keeps PID timing stable.
const unsigned long CONTROL_INTERVAL_MS = 35;
const unsigned long STATUS_INTERVAL_MS = 50;

const bool COMMAND_WATCHDOG_ENABLED = false;
const unsigned long COMMAND_WATCHDOG_TIMEOUT_MS = 500;

// ==================================================
// State
// ==================================================

enum DriveState {
  STATE_IDLE,
  STATE_FOLLOW,
  STATE_RECOVER_LEFT,
  STATE_RECOVER_RIGHT,
  STATE_RECOVER_FORWARD,
  STATE_DERAIL_HOLD,
  STATE_DERAIL_READY,
  STATE_MANUAL_PIVOT_LEFT,
  STATE_MANUAL_PIVOT_RIGHT,
  STATE_RAW_DRIVE,
  STATE_STOPPED,
  STATE_ESTOP
};

enum BranchMode {
  BRANCH_AUTO,
  BRANCH_STRAIGHT,
  BRANCH_LEFT,
  BRANCH_RIGHT
};

DriveState driveState = STATE_IDLE;

BranchMode branchMode = BRANCH_AUTO;

bool followEnabled = false;
bool eStopActive = false;
bool forwardRecoveryAttempted = false;
bool recoveryStopLatched = false;

// ==================================================
// Sensor state
// ==================================================

int offValues[SENSOR_COUNT];
int onValues[SENSOR_COUNT];
int signalValues[SENSOR_COUNT];
int lineStrengthValues[SENSOR_COUNT];
bool lineActiveMask[SENSOR_COUNT];
bool sensorLatchedMask[SENSOR_COUNT];

int currentLinePosition = 0;
int activeSensorCount = 0;
bool validLineForTracking = false;

int clusterCount = 0;
bool junctionCandidate = false;
int selectedClusterIndex = -1;
int selectedClusterStart = -1;
int selectedClusterEnd = -1;
int selectedClusterPosition = 0;
int selectedClusterActiveCount = 0;

int lineLostFrameCount = 0;
int reacquireFrameCount = 0;

// ==================================================
// Recovery hint state
// ==================================================

int recoveryHintDirection = 0;
unsigned long recoveryHintTimeMs = 0;

int recoveryCandidateDirection = 0;
int recoveryCandidateFrames = 0;

// ==================================================
// PID state
// ==================================================

float pidIntegral = 0.0;
float lastError = 0.0;

// ==================================================
// Manual movement
// ==================================================

int manualPivotPwm = 0;

int rawLeftCommand = 0;
int rawRightCommand = 0;

// ==================================================
// Motor command memory
// ==================================================

int lastLeftCommand = 0;
int lastRightCommand = 0;

// ==================================================
// Encoder state
// ==================================================

volatile long leftTicks = 0;
volatile long rightTicks = 0;

volatile byte lastLeftEncoderState = 0;
volatile byte lastRightEncoderState = 0;

// ==================================================
// Wheel speed-control state
// ==================================================

long lastSpeedLeftTicks = 0;
long lastSpeedRightTicks = 0;
unsigned long lastWheelSpeedSampleTimeMs = 0;
bool wheelSpeedSampleValid = false;

float leftMeasuredTicksPerSecond = 0.0f;
float rightMeasuredTicksPerSecond = 0.0f;
float leftMeasuredSpeedMmS = 0.0f;
float rightMeasuredSpeedMmS = 0.0f;
float wheelSpeedSamplePeriodSeconds =
  (float)CONTROL_INTERVAL_MS / 1000.0f;

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

// ==================================================
// Serial
// ==================================================

String serialBuffer = "";

// ==================================================
// Loop timing
// ==================================================

unsigned long lastControlTime = 0;
unsigned long lastStatusTime = 0;
unsigned long turnRecoveryStartTime = 0;
unsigned long lastValidCommandTime = 0;
unsigned long branchModeSetTimeMs = 0;

// ==================================================
// Encoder ISR helpers
// ==================================================

void updateLeftEncoder() {
  byte state = 0;

  if (digitalRead(LEFT_ENC_A_PIN)) {
    state |= 0b10;
  }

  if (digitalRead(LEFT_ENC_B_PIN)) {
    state |= 0b01;
  }

  byte transition = (lastLeftEncoderState << 2) | state;

  int delta = 0;

  if (
    transition == 0b0001 ||
    transition == 0b0111 ||
    transition == 0b1110 ||
    transition == 0b1000
  ) {
    delta = 1;
  } else if (
    transition == 0b0010 ||
    transition == 0b1011 ||
    transition == 0b1101 ||
    transition == 0b0100
  ) {
    delta = -1;
  }

  if (INVERT_LEFT_ENCODER) {
    delta = -delta;
  }

  leftTicks += delta;
  lastLeftEncoderState = state;
}

void updateRightEncoder() {
  byte state = 0;

  if (digitalRead(RIGHT_ENC_A_PIN)) {
    state |= 0b10;
  }

  if (digitalRead(RIGHT_ENC_B_PIN)) {
    state |= 0b01;
  }

  byte transition = (lastRightEncoderState << 2) | state;

  int delta = 0;

  if (
    transition == 0b0001 ||
    transition == 0b0111 ||
    transition == 0b1110 ||
    transition == 0b1000
  ) {
    delta = 1;
  } else if (
    transition == 0b0010 ||
    transition == 0b1011 ||
    transition == 0b1101 ||
    transition == 0b0100
  ) {
    delta = -1;
  }

  if (INVERT_RIGHT_ENCODER) {
    delta = -delta;
  }

  rightTicks += delta;
  lastRightEncoderState = state;
}

// ==================================================
// Setup
// ==================================================

void setup() {
  pinMode(IR_TX_PIN, OUTPUT);
  setIrEmitter(false);

  setupWarningRelayOutputs();

  pinMode(LEFT_RPWM_PIN, OUTPUT);
  pinMode(LEFT_LPWM_PIN, OUTPUT);
  pinMode(RIGHT_RPWM_PIN, OUTPUT);
  pinMode(RIGHT_LPWM_PIN, OUTPUT);

  pinMode(LEFT_ENC_A_PIN, INPUT_PULLUP);
  pinMode(LEFT_ENC_B_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENC_A_PIN, INPUT_PULLUP);
  pinMode(RIGHT_ENC_B_PIN, INPUT_PULLUP);

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);

  lastLeftEncoderState = 0;
  if (digitalRead(LEFT_ENC_A_PIN)) {
    lastLeftEncoderState |= 0b10;
  }
  if (digitalRead(LEFT_ENC_B_PIN)) {
    lastLeftEncoderState |= 0b01;
  }

  lastRightEncoderState = 0;
  if (digitalRead(RIGHT_ENC_A_PIN)) {
    lastRightEncoderState |= 0b10;
  }
  if (digitalRead(RIGHT_ENC_B_PIN)) {
    lastRightEncoderState |= 0b01;
  }

  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_A_PIN), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LEFT_ENC_B_PIN), updateLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_A_PIN), updateRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENC_B_PIN), updateRightEncoder, CHANGE);

  stopMotors();

  Serial.begin(115200);
  serialBuffer.reserve(100);

  driveState = STATE_IDLE;
  followEnabled = false;
  eStopActive = false;
  lastValidCommandTime = millis();

  clearRecoveryHint();
}

// ==================================================
// Main loop
// ==================================================

void loop() {
  processSerial();

  unsigned long now = millis();

  applyCommandWatchdog(now);

  if (now - lastControlTime >= CONTROL_INTERVAL_MS) {
    lastControlTime = now;
    runControlLoop();
  }

  if (now - lastStatusTime >= STATUS_INTERVAL_MS) {
    lastStatusTime = now;
    reportStatus();
  }
}

// ==================================================
// IR emitter
// ==================================================

void setIrEmitter(bool on) {
  if (IR_CONTROL_ACTIVE_HIGH) {
    digitalWrite(IR_TX_PIN, on ? HIGH : LOW);
  } else {
    digitalWrite(IR_TX_PIN, on ? LOW : HIGH);
  }
}

// ==================================================
// Warning relay outputs
// ==================================================

void writeRelayOutput(int pin, bool active) {
  if (WARNING_RELAY_ACTIVE_LOW) {
    digitalWrite(pin, active ? LOW : HIGH);
  } else {
    digitalWrite(pin, active ? HIGH : LOW);
  }
}

void prepareRelayOutput(int pin) {
  writeRelayOutput(pin, false);
  pinMode(pin, OUTPUT);
  writeRelayOutput(pin, false);
}

void setupWarningRelayOutputs() {
  prepareRelayOutput(WARNING_RELAY_LIGHT_PIN);
  prepareRelayOutput(WARNING_RELAY_SPARE_1_PIN);
  prepareRelayOutput(WARNING_RELAY_SPARE_2_PIN);
  prepareRelayOutput(WARNING_RELAY_SPARE_3_PIN);
}

void setWarningLight(bool active) {
  writeRelayOutput(WARNING_RELAY_LIGHT_PIN, active);
}

void updateWarningLightFromMotorCommands() {
  bool motorsCommanded =
    lastLeftCommand != 0 ||
    lastRightCommand != 0;

  setWarningLight(motorsCommanded);
}

// ==================================================
// Line cluster selection
// ==================================================

void clearLineSelectionState() {
  activeSensorCount = 0;
  validLineForTracking = false;
  currentLinePosition = 0;

  clusterCount = 0;
  junctionCandidate = false;

  selectedClusterIndex = -1;
  selectedClusterStart = -1;
  selectedClusterEnd = -1;
  selectedClusterPosition = 0;
  selectedClusterActiveCount = 0;
}

void expireBranchCommandIfNeeded() {
  // A failed recovery must preserve the Pi-selected branch for the supervised
  // C:REACQUIRE_LINE attempt.  RESET, ESTOP and an explicit CLEAR still clear it.
  if (branchMode == BRANCH_AUTO || recoveryStopLatched) {
    return;
  }

  if (millis() - branchModeSetTimeMs <= BRANCH_COMMAND_TIMEOUT_MS) {
    return;
  }

  branchMode = BRANCH_AUTO;
  branchModeSetTimeMs = 0;
}

void updateClusterTelemetry() {
  clusterCount = 0;
  junctionCandidate = false;

  int widestCluster = 0;
  int i = 0;

  while (i < SENSOR_COUNT) {
    while (i < SENSOR_COUNT && !lineActiveMask[i]) {
      i++;
    }

    if (i >= SENSOR_COUNT) {
      break;
    }

    int count = 0;

    while (i < SENSOR_COUNT && lineActiveMask[i]) {
      count++;
      i++;
    }

    clusterCount++;

    if (count > widestCluster) {
      widestCluster = count;
    }
  }

  junctionCandidate =
      clusterCount > 1 ||
      widestCluster >= JUNCTION_WIDE_CLUSTER_MIN;
}

bool acceptSelectedLine(
  long weightedSum,
  long totalSignal,
  int count,
  int clusterIndex,
  int startIndex,
  int endIndex
) {
  if (count < LINE_ACTIVE_MIN || totalSignal < LINE_TOTAL_STRENGTH_MIN) {
    return false;
  }

  currentLinePosition = weightedSum / totalSignal;
  activeSensorCount = count;
  validLineForTracking = true;

  selectedClusterIndex = clusterIndex;
  selectedClusterStart = startIndex;
  selectedClusterEnd = endIndex;
  selectedClusterPosition = currentLinePosition;
  selectedClusterActiveCount = count;

  return true;
}

bool chooseClusterInRange(int mode, int firstIndex, int lastIndex) {
  firstIndex = constrain(firstIndex, 0, SENSOR_COUNT - 1);
  lastIndex = constrain(lastIndex, 0, SENSOR_COUNT - 1);

  if (firstIndex > lastIndex) {
    return false;
  }

  bool haveBest = false;
  int bestClusterIndex = -1;
  int bestStart = -1;
  int bestEnd = -1;
  int bestCount = 0;
  int bestPosition = 0;
  long bestWeightedSum = 0;
  long bestTotalSignal = 0;

  int clusterIndex = 0;
  int i = firstIndex;

  while (i <= lastIndex) {
    while (i <= lastIndex && !lineActiveMask[i]) {
      i++;
    }

    if (i > lastIndex) {
      break;
    }

    int startIndex = i;
    int count = 0;
    long weightedSum = 0;
    long totalSignal = 0;

    while (i <= lastIndex && lineActiveMask[i]) {
      count++;
      weightedSum += (long)lineStrengthValues[i] * SENSOR_WEIGHTS[i];
      totalSignal += lineStrengthValues[i];
      i++;
    }

    int endIndex = i - 1;

    if (totalSignal > 0) {
      int position = weightedSum / totalSignal;
      bool better = false;

      if (!haveBest) {
        better = true;
      } else if (mode == BRANCH_LEFT) {
        better =
            position < bestPosition ||
            (position == bestPosition && totalSignal > bestTotalSignal);
      } else if (mode == BRANCH_RIGHT) {
        better =
            position > bestPosition ||
            (position == bestPosition && totalSignal > bestTotalSignal);
      } else if (mode == BRANCH_STRAIGHT) {
        better =
            abs(position) < abs(bestPosition) ||
            (abs(position) == abs(bestPosition) && totalSignal > bestTotalSignal);
      } else {
        better =
            totalSignal > bestTotalSignal ||
            (totalSignal == bestTotalSignal && abs(position) < abs(bestPosition));
      }

      if (better) {
        haveBest = true;
        bestClusterIndex = clusterIndex;
        bestStart = startIndex;
        bestEnd = endIndex;
        bestCount = count;
        bestPosition = position;
        bestWeightedSum = weightedSum;
        bestTotalSignal = totalSignal;
      }
    }

    clusterIndex++;
  }

  if (!haveBest) {
    return false;
  }

  return acceptSelectedLine(
    bestWeightedSum,
    bestTotalSignal,
    bestCount,
    bestClusterIndex,
    bestStart,
    bestEnd
  );
}

void selectLineForTracking() {
  clearLineSelectionState();
  expireBranchCommandIfNeeded();
  updateClusterTelemetry();

  bool selected = false;

  if (branchMode == BRANCH_LEFT) {
    selected = chooseClusterInRange(
      BRANCH_LEFT,
      LEFT_BRANCH_FIRST_INDEX,
      LEFT_BRANCH_LAST_INDEX
    );
  } else if (branchMode == BRANCH_RIGHT) {
    selected = chooseClusterInRange(
      BRANCH_RIGHT,
      RIGHT_BRANCH_FIRST_INDEX,
      RIGHT_BRANCH_LAST_INDEX
    );
  } else if (branchMode == BRANCH_STRAIGHT) {
    selected = chooseClusterInRange(
      BRANCH_STRAIGHT,
      STRAIGHT_BRANCH_FIRST_INDEX,
      STRAIGHT_BRANCH_LAST_INDEX
    );

    if (!selected) {
      selected = chooseClusterInRange(
        BRANCH_STRAIGHT,
        STRAIGHT_BRANCH_FALLBACK_FIRST_INDEX,
        STRAIGHT_BRANCH_FALLBACK_LAST_INDEX
      );
    }
  }

  if (selected) {
    return;
  }

  if (branchMode == BRANCH_STRAIGHT) {
    return;
  }

  if (junctionCandidate) {
    selected = chooseClusterInRange(
      BRANCH_STRAIGHT,
      STRAIGHT_BRANCH_FALLBACK_FIRST_INDEX,
      STRAIGHT_BRANCH_FALLBACK_LAST_INDEX
    );

    if (selected) {
      return;
    }
  }

  BranchMode fallbackMode = BRANCH_AUTO;

  if (branchMode == BRANCH_LEFT || branchMode == BRANCH_RIGHT) {
    fallbackMode = branchMode;
  }

  chooseClusterInRange(fallbackMode, 0, SENSOR_COUNT - 1);
}

// ==================================================
// Sensor reading
// ==================================================

void readSensorsSwitching() {
  long offSums[SENSOR_COUNT];
  long onSums[SENSOR_COUNT];

  for (int i = 0; i < SENSOR_COUNT; i++) {
    offSums[i] = 0;
    onSums[i] = 0;
  }

  setIrEmitter(false);
  delayMicroseconds(IR_SETTLE_DELAY_US);

  for (int sample = 0; sample < ANALOG_SAMPLES; sample++) {
    for (int i = 0; i < SENSOR_COUNT; i++) {
      analogRead(SENSOR_PINS[i]);
      offSums[i] += analogRead(SENSOR_PINS[i]);
    }
    delayMicroseconds(ANALOG_SAMPLE_DELAY_US);
  }

  setIrEmitter(true);
  delayMicroseconds(IR_SETTLE_DELAY_US);

  for (int sample = 0; sample < ANALOG_SAMPLES; sample++) {
    for (int i = 0; i < SENSOR_COUNT; i++) {
      analogRead(SENSOR_PINS[i]);
      onSums[i] += analogRead(SENSOR_PINS[i]);
    }
    delayMicroseconds(ANALOG_SAMPLE_DELAY_US);
  }

  setIrEmitter(false);

  int maxSignal = 0;
  int minSignal = 1023;

  for (int i = 0; i < SENSOR_COUNT; i++) {
    offValues[i] = offSums[i] / ANALOG_SAMPLES;
    onValues[i] = onSums[i] / ANALOG_SAMPLES;

    int signal = offValues[i] - onValues[i];

    if (signal < 0) {
      signal = 0;
    }

    signalValues[i] = signal;
    lineStrengthValues[i] = 0;
    lineActiveMask[i] = false;

    if (signalValues[i] > maxSignal) {
      maxSignal = signalValues[i];
    }

    if (signalValues[i] < minSignal) {
      minSignal = signalValues[i];
    }
  }

  // The early quality gates below must never leave the previous frame marked
  // valid after the emitter/sensor signal disappears.
  clearLineSelectionState();

  if (maxSignal < MIN_MAX_SIGNAL) {
    return;
  }

  int contrast = maxSignal - minSignal;

  if (contrast < MIN_CONTRAST) {
    return;
  }

  for (int i = 0; i < SENSOR_COUNT; i++) {
    int threshold = SENSOR_SIGNAL_THRESHOLD[i];

    if (sensorLatchedMask[i]) {
      if (signalValues[i] < threshold - SENSOR_RELEASE_HYSTERESIS) {
        sensorLatchedMask[i] = false;
      }
    } else if (signalValues[i] >= threshold) {
      sensorLatchedMask[i] = true;
    }

    if (sensorLatchedMask[i]) {
      // Keep a small positive contribution while the channel is held by the
      // release hysteresis, so a valid cluster does not flicker at threshold.
      lineStrengthValues[i] = max(
        1,
        signalValues[i] - (threshold - SENSOR_RELEASE_HYSTERESIS)
      );
      lineActiveMask[i] = true;
    }
  }

  selectLineForTracking();
}

// ==================================================
// Follow-only encoder speed control
// ==================================================

void updateWheelSpeedMeasurement(unsigned long now) {
  long leftCopy = 0;
  long rightCopy = 0;

  noInterrupts();
  leftCopy = leftTicks;
  rightCopy = rightTicks;
  interrupts();

  if (!wheelSpeedSampleValid) {
    lastSpeedLeftTicks = leftCopy;
    lastSpeedRightTicks = rightCopy;
    lastWheelSpeedSampleTimeMs = now;
    wheelSpeedSampleValid = true;
    return;
  }

  unsigned long elapsedMs = now - lastWheelSpeedSampleTimeMs;

  if (elapsedMs == 0) {
    return;
  }

  long leftDelta = leftCopy - lastSpeedLeftTicks;
  long rightDelta = rightCopy - lastSpeedRightTicks;

  float seconds = (float)elapsedMs / 1000.0f;
  wheelSpeedSamplePeriodSeconds = seconds;

  leftMeasuredTicksPerSecond = (float)leftDelta / seconds;
  rightMeasuredTicksPerSecond = (float)rightDelta / seconds;

  leftMeasuredSpeedMmS = leftMeasuredTicksPerSecond / LEFT_TICKS_PER_MM;
  rightMeasuredSpeedMmS = rightMeasuredTicksPerSecond / RIGHT_TICKS_PER_MM;

  lastSpeedLeftTicks = leftCopy;
  lastSpeedRightTicks = rightCopy;
  lastWheelSpeedSampleTimeMs = now;
}

void resetWheelSpeedControl() {
  leftTargetSpeedMmS = 0.0f;
  rightTargetSpeedMmS = 0.0f;
  leftSpeedErrorMmS = 0.0f;
  rightSpeedErrorMmS = 0.0f;
  leftSpeedIntegral = 0.0f;
  rightSpeedIntegral = 0.0f;
  leftLastSpeedError = 0.0f;
  rightLastSpeedError = 0.0f;
  leftWheelPwmCorrection = 0.0f;
  rightWheelPwmCorrection = 0.0f;
}

float pwmToFollowSpeedMmS(int pwmMagnitude) {
  pwmMagnitude = constrain(
    pwmMagnitude,
    0,
    ABSOLUTE_MAX_MOTOR_PWM
  );

  if (pwmMagnitude == 0) {
    return 0.0f;
  }

  if (pwmMagnitude <= SPEED_REFERENCE_PWM_LOW) {
    return
      FOLLOW_SPEED_MM_S_AT_PWM_40 *
      ((float)pwmMagnitude / SPEED_REFERENCE_PWM_LOW);
  }

  if (pwmMagnitude <= SPEED_REFERENCE_PWM_HIGH) {
    float fraction =
      (float)(pwmMagnitude - SPEED_REFERENCE_PWM_LOW) /
      (SPEED_REFERENCE_PWM_HIGH - SPEED_REFERENCE_PWM_LOW);

    return
      FOLLOW_SPEED_MM_S_AT_PWM_40 +
      fraction * (
        FOLLOW_SPEED_MM_S_AT_PWM_80 -
        FOLLOW_SPEED_MM_S_AT_PWM_40
      );
  }

  return FOLLOW_SPEED_MM_S_AT_PWM_80;
}

int applySingleWheelSpeedControl(
  int openLoopCommand,
  float measuredSpeedMmS,
  float &targetSpeedMmS,
  float &speedErrorMmS,
  float &speedIntegral,
  float &lastSpeedError,
  float &pwmCorrection
) {
  int direction = 0;

  if (openLoopCommand > 0) {
    direction = 1;
  } else if (openLoopCommand < 0) {
    direction = -1;
  } else {
    targetSpeedMmS = 0.0f;
    speedErrorMmS = 0.0f;
    speedIntegral = 0.0f;
    lastSpeedError = 0.0f;
    pwmCorrection = 0.0f;
    return 0;
  }

  int openLoopMagnitude = abs(openLoopCommand);
  targetSpeedMmS = pwmToFollowSpeedMmS(openLoopMagnitude);

  // Compare speeds in the requested travel direction.  This preserves signed
  // encoder conventions if a future FOLLOW mode allows reverse commands.
  float measuredAlongCommandMmS = measuredSpeedMmS * direction;
  speedErrorMmS = targetSpeedMmS - measuredAlongCommandMmS;

  float dtSeconds = wheelSpeedSamplePeriodSeconds;

  if (dtSeconds <= 0.0f) {
    dtSeconds = (float)CONTROL_INTERVAL_MS / 1000.0f;
  }
  speedIntegral += speedErrorMmS * dtSeconds;
  speedIntegral = constrain(
    speedIntegral,
    -WHEEL_SPEED_INTEGRAL_LIMIT,
    WHEEL_SPEED_INTEGRAL_LIMIT
  );

  float derivative =
    (speedErrorMmS - lastSpeedError) / dtSeconds;

  pwmCorrection =
    (wheelSpeedKp * speedErrorMmS) +
    (wheelSpeedKi * speedIntegral) +
    (wheelSpeedKd * derivative);

  pwmCorrection = constrain(
    pwmCorrection,
    -MAX_WHEEL_SPEED_CORRECTION_PWM,
    MAX_WHEEL_SPEED_CORRECTION_PWM
  );

  lastSpeedError = speedErrorMmS;

  int correctedMagnitude = openLoopMagnitude + (int)pwmCorrection;
  correctedMagnitude = constrain(
    correctedMagnitude,
    0,
    ABSOLUTE_MAX_MOTOR_PWM
  );

  return direction * correctedMagnitude;
}

void setFollowDriveCommand(int leftCommand, int rightCommand) {
  if (!WHEEL_SPEED_CONTROL_ENABLED || !wheelSpeedSampleValid) {
    resetWheelSpeedControl();
    setDriveCommand(leftCommand, rightCommand);
    return;
  }

  int correctedLeft = applySingleWheelSpeedControl(
    leftCommand,
    leftMeasuredSpeedMmS,
    leftTargetSpeedMmS,
    leftSpeedErrorMmS,
    leftSpeedIntegral,
    leftLastSpeedError,
    leftWheelPwmCorrection
  );

  int correctedRight = applySingleWheelSpeedControl(
    rightCommand,
    rightMeasuredSpeedMmS,
    rightTargetSpeedMmS,
    rightSpeedErrorMmS,
    rightSpeedIntegral,
    rightLastSpeedError,
    rightWheelPwmCorrection
  );

  setDriveCommand(correctedLeft, correctedRight);
}

void runControlLoop() {
  updateWheelSpeedMeasurement(millis());

  if (eStopActive) {
    resetWheelSpeedControl();
    stopMotors();
    driveState = STATE_ESTOP;
    return;
  }

  // Do not spend a full switching-IR scan while stopped, in RAW_DRIVE or while
  // an IMU/manual pivot owns the motors.  Normal line following and recovery
  // still use the exact same IR acquisition path.
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

  // A failed automatic recovery is a controlled stationary hold, not a
  // terminal fault. Keep scanning so a person can realign the AGV, but never
  // command motion until the Pi has completed its safety authorization.
  if (
    driveState == STATE_DERAIL_HOLD ||
    driveState == STATE_DERAIL_READY
  ) {
    resetWheelSpeedControl();
    readSensorsSwitching();
    handleDerailHoldState();
    return;
  }

  if (!followEnabled) {
    // Keep acquiring IR data while stationary so IDLE is a safe diagnostic
    // state.  This only refreshes telemetry; motors remain stopped and a
    // detected line never starts the AGV by itself.
    readSensorsSwitching();

    resetWheelSpeedControl();
    if (driveState != STATE_STOPPED) {
      driveState = STATE_IDLE;
    }

    stopMotors();
    return;
  }

  readSensorsSwitching();

  if (
    driveState == STATE_RECOVER_FORWARD ||
    driveState == STATE_RECOVER_LEFT ||
    driveState == STATE_RECOVER_RIGHT
  ) {
    resetWheelSpeedControl();
    handleRecoveryState();
    return;
  }

  if (validLineForTracking) {
    lineLostFrameCount = 0;
    reacquireFrameCount = 0;

    updateRecoveryHintFromCurrentLine();

    driveState = STATE_FOLLOW;
    applyLinePid();
    return;
  }

  resetWheelSpeedControl();
  lineLostFrameCount++;

  if (lineLostFrameCount < LINE_LOST_RECOVERY_FRAMES) {
    setDriveCommand(LINE_LOST_HOLD_PWM, LINE_LOST_HOLD_PWM);
    return;
  }

  if (forwardRecoveryAttempted) {
    stopDueToNoFreshRecoveryHint();
    return;
  }

  startForwardRecovery();
  handleRecoveryState();
}

// ==================================================
// PID line following
// ==================================================

void applyLinePid() {
  int pidLinePosition = currentLinePosition;

  if (branchMode == BRANCH_STRAIGHT) {
    pidLinePosition = constrain(
      pidLinePosition,
      -STRAIGHT_BRANCH_POSITION_LIMIT,
      STRAIGHT_BRANCH_POSITION_LIMIT
    );
  } else if (branchMode == BRANCH_LEFT || branchMode == BRANCH_RIGHT) {
    pidLinePosition = constrain(
      pidLinePosition,
      -TURN_BRANCH_POSITION_LIMIT,
      TURN_BRANCH_POSITION_LIMIT
    );
  }

  float error = (float)pidLinePosition;

  if (INVERT_STEERING) {
    error = -error;
  }

  if (Ki != 0.0f) {
    pidIntegral += error;

    if (pidIntegral > INTEGRAL_LIMIT) {
      pidIntegral = INTEGRAL_LIMIT;
    } else if (pidIntegral < -INTEGRAL_LIMIT) {
      pidIntegral = -INTEGRAL_LIMIT;
    }
  } else {
    pidIntegral = 0.0f;
  }

  float derivative = error - lastError;
  lastError = error;

  derivative = constrain(
    derivative,
    -MAX_DERIVATIVE_STEP,
    MAX_DERIVATIVE_STEP
  );

  float steering = (Kp * error) + (Ki * pidIntegral) + (Kd * derivative);

  bool sharpCornerCandidate = abs((int)error) >= CORNER_DETECT_POSITION;

  int steeringLimit = MAX_NORMAL_LINE_STEERING;
  int minimumCommand = 0;

  if (sharpCornerCandidate) {
    steeringLimit = MAX_CORNER_LINE_STEERING;
    minimumCommand = -MAX_CORNER_REVERSE_PWM;
  }

  if (branchMode == BRANCH_STRAIGHT) {
    steeringLimit = STRAIGHT_BRANCH_STEERING_LIMIT;
  } else if (branchMode == BRANCH_LEFT || branchMode == BRANCH_RIGHT) {
    steeringLimit = TURN_BRANCH_STEERING_LIMIT;
  }

  steering = constrain(
    steering,
    -steeringLimit,
    steeringLimit
  );

  int followSpeed = baseSpeed;

  if (branchMode == BRANCH_LEFT || branchMode == BRANCH_RIGHT) {
    followSpeed = min(baseSpeed, ACTIVE_BRANCH_SPEED);
  }

  int leftCommand = followSpeed + (int)steering;
  int rightCommand = followSpeed - (int)steering;

  leftCommand = constrain(
    leftCommand,
    minimumCommand,
    ABSOLUTE_MAX_MOTOR_PWM
  );

  rightCommand = constrain(
    rightCommand,
    minimumCommand,
    ABSOLUTE_MAX_MOTOR_PWM
  );

  // Only normal line-following uses encoder speed control.  RAW_DRIVE,
  // manual/IMU pivots, recovery and all stop paths still call setDriveCommand.
  setFollowDriveCommand(leftCommand, rightCommand);
}

void resetPID() {
  pidIntegral = 0.0;
  lastError = 0.0;
}

// ==================================================
// Recovery hint logic
// ==================================================

void updateRecoveryHintFromCurrentLine() {
  int direction = 0;

  if (currentLinePosition >= RECOVERY_DIRECTION_MIN_POSITION) {
    direction = +1;
  } else if (currentLinePosition <= -RECOVERY_DIRECTION_MIN_POSITION) {
    direction = -1;
  } else {
    recoveryCandidateDirection = 0;
    recoveryCandidateFrames = 0;
    return;
  }

  if (direction == recoveryCandidateDirection) {
    recoveryCandidateFrames++;
  } else {
    recoveryCandidateDirection = direction;
    recoveryCandidateFrames = 1;
  }

  if (recoveryCandidateFrames >= RECOVERY_DIRECTION_CONFIRM_FRAMES) {
    recoveryHintDirection = direction;
    recoveryHintTimeMs = millis();
  }
}

int getFreshRecoveryHint() {
  if (recoveryHintDirection == 0) {
    return 0;
  }

  unsigned long ageMs = millis() - recoveryHintTimeMs;

  if (ageMs > RECOVERY_HINT_MAX_AGE_MS) {
    clearRecoveryHint();
    return 0;
  }

  return recoveryHintDirection;
}

void clearRecoveryHint() {
  recoveryHintDirection = 0;
  recoveryHintTimeMs = 0;
  recoveryCandidateDirection = 0;
  recoveryCandidateFrames = 0;
}

// ==================================================
// Recovery behavior
// ==================================================

void startForwardRecovery() {
  forwardRecoveryAttempted = true;
  driveState = STATE_RECOVER_FORWARD;
  turnRecoveryStartTime = millis();
  reacquireFrameCount = 0;
  resetPID();
}

void finishLineRecovery() {
  driveState = STATE_FOLLOW;
  recoveryStopLatched = false;
  forwardRecoveryAttempted = false;

  lineLostFrameCount = 0;
  reacquireFrameCount = 0;

  clearRecoveryHint();
  resetPID();
  applyLinePid();
}

void latchRecoveryStop() {
  followEnabled = false;
  driveState = STATE_DERAIL_HOLD;
  recoveryStopLatched = true;

  lineLostFrameCount = 0;
  reacquireFrameCount = 0;

  // Preserve both the Pi-selected branch and the last reliable line side for
  // C:REACQUIRE_LINE.  RESET, ESTOP and an explicit CLEAR remain authoritative.
  resetPID();
  brakeAndStopMotors();

  digitalWrite(STATUS_LED, LOW);
}

void handleDerailHoldState() {
  // Every path into this state must remain stationary. The Pi may authorize
  // a later resume only after its own mission, obstacle and safety checks.
  stopMotors();

  bool lineCentered =
    validLineForTracking &&
    abs(currentLinePosition) <= REACQUIRE_POSITION_TOLERANCE;

  if (driveState == STATE_DERAIL_READY) {
    if (!lineCentered) {
      driveState = STATE_DERAIL_HOLD;
      reacquireFrameCount = 0;
    }
    return;
  }

  if (lineCentered) {
    reacquireFrameCount++;
  } else {
    reacquireFrameCount = 0;
  }

  if (reacquireFrameCount >= REACQUIRE_CONFIRM_FRAMES) {
    driveState = STATE_DERAIL_READY;
    digitalWrite(STATUS_LED, HIGH);
  }
}

void handleRecoveryState() {
  bool lineCentered =
    validLineForTracking &&
    abs(currentLinePosition) <= REACQUIRE_POSITION_TOLERANCE;

  if (lineCentered) {
    reacquireFrameCount++;
  } else {
    reacquireFrameCount = 0;
  }

  if (reacquireFrameCount >= REACQUIRE_CONFIRM_FRAMES) {
    finishLineRecovery();
    return;
  }

  unsigned long elapsed = millis() - turnRecoveryStartTime;

  if (driveState == STATE_RECOVER_FORWARD) {
    if (elapsed < FORWARD_RECOVERY_TIME_MS) {
      setDriveCommand(FORWARD_RECOVERY_PWM, FORWARD_RECOVERY_PWM);
      return;
    }

    int direction = getFreshRecoveryHint();

    if (direction > 0) {
      driveState = STATE_RECOVER_RIGHT;
    } else if (direction < 0) {
      driveState = STATE_RECOVER_LEFT;
    } else {
      latchRecoveryStop();
      return;
    }

    turnRecoveryStartTime = millis();
    reacquireFrameCount = 0;
    elapsed = 0;
  }

  if (elapsed >= MAX_TURN_RECOVERY_TIME_MS) {
    latchRecoveryStop();
    return;
  }

  if (driveState == STATE_RECOVER_RIGHT) {
    setDriveCommand(TURN_RECOVERY_PWM, -TURN_RECOVERY_PWM);
  } else if (driveState == STATE_RECOVER_LEFT) {
    setDriveCommand(-TURN_RECOVERY_PWM, TURN_RECOVERY_PWM);
  } else {
    latchRecoveryStop();
  }
}

// ==================================================
// Motor control
// ==================================================

void setDriveCommand(int leftCommand, int rightCommand) {
  leftCommand = constrain(
    leftCommand,
    -ABSOLUTE_MAX_MOTOR_PWM,
    ABSOLUTE_MAX_MOTOR_PWM
  );

  rightCommand = constrain(
    rightCommand,
    -ABSOLUTE_MAX_MOTOR_PWM,
    ABSOLUTE_MAX_MOTOR_PWM
  );

  lastLeftCommand = slewMotorCommand(lastLeftCommand, leftCommand);
  lastRightCommand = slewMotorCommand(lastRightCommand, rightCommand);

  lastLeftCommand = constrain(
    lastLeftCommand,
    -ABSOLUTE_MAX_MOTOR_PWM,
    ABSOLUTE_MAX_MOTOR_PWM
  );

  lastRightCommand = constrain(
    lastRightCommand,
    -ABSOLUTE_MAX_MOTOR_PWM,
    ABSOLUTE_MAX_MOTOR_PWM
  );

  writeSingleMotor(
    LEFT_RPWM_PIN,
    LEFT_LPWM_PIN,
    lastLeftCommand,
    INVERT_LEFT_MOTOR
  );

  writeSingleMotor(
    RIGHT_RPWM_PIN,
    RIGHT_LPWM_PIN,
    lastRightCommand,
    INVERT_RIGHT_MOTOR
  );

  updateWarningLightFromMotorCommands();
}

int slewMotorCommand(int applied, int target) {
  if (applied == target) {
    return applied;
  }

  if ((applied > 0 && target < 0) || (applied < 0 && target > 0)) {
    target = 0;
  }

  int step = PWM_RISE_PER_CONTROL;

  if (abs(target) < abs(applied)) {
    step = PWM_FALL_PER_CONTROL;
  }

  if (target > applied) {
    return min(applied + step, target);
  }

  return max(applied - step, target);
}

void writeSingleMotor(int rpwmPin, int lpwmPin, int command, bool invertMotor) {
  if (invertMotor) {
    command = -command;
  }

  command = constrain(
    command,
    -ABSOLUTE_MAX_MOTOR_PWM,
    ABSOLUTE_MAX_MOTOR_PWM
  );

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

void coastMotorOutputs() {
  analogWrite(LEFT_RPWM_PIN, 0);
  analogWrite(LEFT_LPWM_PIN, 0);
  analogWrite(RIGHT_RPWM_PIN, 0);
  analogWrite(RIGHT_LPWM_PIN, 0);
  setWarningLight(false);
}

void applyBrakePulseToMotor(int rpwmPin, int lpwmPin) {
  analogWrite(rpwmPin, ACTIVE_BRAKE_PWM);
  analogWrite(lpwmPin, ACTIVE_BRAKE_PWM);
}

void brakeMotorOutputsBriefly() {
  setWarningLight(true);
  applyBrakePulseToMotor(LEFT_RPWM_PIN, LEFT_LPWM_PIN);
  applyBrakePulseToMotor(RIGHT_RPWM_PIN, RIGHT_LPWM_PIN);
  delay(ACTIVE_BRAKE_TIME_MS);
  coastMotorOutputs();
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
  bool hadMotorCommand =
    lastLeftCommand != 0 ||
    lastRightCommand != 0 ||
    rawLeftCommand != 0 ||
    rawRightCommand != 0 ||
    manualPivotPwm != 0;

  clearMotorCommandMemory();
  manualPivotPwm = 0;

  if (ACTIVE_BRAKE_ON_STOP && hadMotorCommand) {
    brakeMotorOutputsBriefly();
  } else {
    coastMotorOutputs();
  }
}

void stopDueToNoFreshRecoveryHint() {
  latchRecoveryStop();
}

// ==================================================
// Serial command handling
// ==================================================

void processSerial() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      if (serialBuffer.length() > 0) {
        String command = serialBuffer;
        serialBuffer = "";
        command.trim();
        processCommand(command);
      }
    } else {
      if (serialBuffer.length() < 100) {
        serialBuffer += c;
      } else {
        serialBuffer = "";
      }
    }
  }
}

void processCommand(String command) {
  command.trim();

  if (command.length() == 0) {
    return;
  }

  if (command == "C:START") {
    noteValidCommand();
    commandStart();
    return;
  }

  if (command == "C:STOP") {
    noteValidCommand();
    commandStop();
    return;
  }

  if (command == "C:ESTOP") {
    noteValidCommand();
    commandEstop();
    return;
  }

  if (command == "C:RESET") {
    noteValidCommand();
    commandReset();
    return;
  }

  if (command == "C:RESET_TICKS") {
    noteValidCommand();
    commandResetTicks();
    return;
  }

  if (command.startsWith("C:SET_SPEED,")) {
    noteValidCommand();
    int value = command.substring(12).toInt();
    commandSetSpeed(value);
    return;
  }

  if (command.startsWith("C:SET_PID,")) {
    noteValidCommand();
    commandSetPid(command.substring(10));
    return;
  }

  if (command.startsWith("C:SET_BRANCH,")) {
    noteValidCommand();
    commandSetBranch(command.substring(13));
    return;
  }

  if (command == "C:CLEAR_BRANCH") {
    noteValidCommand();
    commandClearBranch();
    return;
  }

  if (command.startsWith("C:PIVOT_LEFT,")) {
    noteValidCommand();
    int pwm = command.substring(13).toInt();
    commandPivotLeft(pwm);
    return;
  }

  if (command.startsWith("C:PIVOT_RIGHT,")) {
    noteValidCommand();
    int pwm = command.substring(14).toInt();
    commandPivotRight(pwm);
    return;
  }

  if (command.startsWith("C:RAW_DRIVE,")) {
    noteValidCommand();
    commandRawDrive(command.substring(12));
    return;
  }

  if (command == "C:REACQUIRE_LINE") {
    noteValidCommand();
    commandReacquireLine();
    return;
  }

  if (command == "C:AUTHORIZE_DERAIL_RESUME") {
    noteValidCommand();
    commandAuthorizeDerailResume();
    return;
  }
}

void noteValidCommand() {
  lastValidCommandTime = millis();
}

void applyCommandWatchdog(unsigned long now) {
  if (!COMMAND_WATCHDOG_ENABLED) {
    return;
  }

  bool motorsMayBeActive =
    followEnabled ||
    driveState == STATE_RAW_DRIVE ||
    driveState == STATE_MANUAL_PIVOT_LEFT ||
    driveState == STATE_MANUAL_PIVOT_RIGHT;

  if (
    motorsMayBeActive &&
    (now - lastValidCommandTime > COMMAND_WATCHDOG_TIMEOUT_MS)
  ) {
    commandStop();
  }
}

void commandStart() {
  if (eStopActive || recoveryStopLatched) {
    return;
  }

  // START can be published more than once by the supervisory stack.  A repeat
  // must not reset PID/lost-frame/recovery state while the Arduino is already
  // following or recovering a line.
  if (
    followEnabled &&
    (
      driveState == STATE_FOLLOW ||
      driveState == STATE_RECOVER_FORWARD ||
      driveState == STATE_RECOVER_LEFT ||
      driveState == STATE_RECOVER_RIGHT
    )
  ) {
    return;
  }

  rawLeftCommand = 0;
  rawRightCommand = 0;

  followEnabled = true;
  driveState = STATE_FOLLOW;
  forwardRecoveryAttempted = false;

  lineLostFrameCount = 0;
  reacquireFrameCount = 0;

  clearRecoveryHint();
  resetPID();

  digitalWrite(STATUS_LED, HIGH);
}

void commandReacquireLine() {
  if (eStopActive) {
    return;
  }

  recoveryStopLatched = false;

  // The branch may have been held while STOPPED for longer than the normal
  // command timeout.  Resume the same Pi-selected context for this one
  // supervised recovery attempt.
  if (branchMode != BRANCH_AUTO) {
    branchModeSetTimeMs = millis();
  }

  rawLeftCommand = 0;
  rawRightCommand = 0;
  manualPivotPwm = 0;

  followEnabled = true;
  lineLostFrameCount = 0;
  reacquireFrameCount = 0;

  // Do not discard the preserved last-side hint.  If it has become too old,
  // getFreshRecoveryHint() will safely refuse to pivot after the forward creep.
  resetPID();
  startForwardRecovery();

  digitalWrite(STATUS_LED, HIGH);
}

void commandAuthorizeDerailResume() {
  if (
    eStopActive ||
    !recoveryStopLatched ||
    driveState != STATE_DERAIL_READY
  ) {
    return;
  }

  // Refuse authorization if the line disappeared after DERAIL_READY was
  // reported but before the Pi's five-second safety delay completed.
  if (
    !validLineForTracking ||
    abs(currentLinePosition) > REACQUIRE_POSITION_TOLERANCE
  ) {
    driveState = STATE_DERAIL_HOLD;
    reacquireFrameCount = 0;
    return;
  }

  recoveryStopLatched = false;
  followEnabled = true;
  driveState = STATE_FOLLOW;
  forwardRecoveryAttempted = false;
  lineLostFrameCount = 0;
  reacquireFrameCount = 0;

  resetPID();
  digitalWrite(STATUS_LED, HIGH);
}

void commandStop() {
  followEnabled = false;

  rawLeftCommand = 0;
  rawRightCommand = 0;
  manualPivotPwm = 0;

  if (!eStopActive) {
    driveState = STATE_IDLE;
  }

  lineLostFrameCount = 0;
  reacquireFrameCount = 0;
  forwardRecoveryAttempted = false;

  clearRecoveryHint();
  commandClearBranch();
  resetPID();
  brakeAndStopMotors();

  digitalWrite(STATUS_LED, LOW);
}

void commandEstop() {
  eStopActive = true;
  followEnabled = false;
  driveState = STATE_ESTOP;
  recoveryStopLatched = true;

  rawLeftCommand = 0;
  rawRightCommand = 0;
  manualPivotPwm = 0;

  lineLostFrameCount = 0;
  reacquireFrameCount = 0;
  forwardRecoveryAttempted = false;

  clearRecoveryHint();
  commandClearBranch();
  resetPID();
  brakeAndStopMotors();

  digitalWrite(STATUS_LED, LOW);
}

void commandReset() {
  eStopActive = false;
  followEnabled = false;
  driveState = STATE_IDLE;
  recoveryStopLatched = false;

  rawLeftCommand = 0;
  rawRightCommand = 0;
  manualPivotPwm = 0;

  lineLostFrameCount = 0;
  reacquireFrameCount = 0;
  forwardRecoveryAttempted = false;

  clearRecoveryHint();
  commandClearBranch();
  resetPID();
  stopMotors();

  digitalWrite(STATUS_LED, LOW);
}

void commandResetTicks() {
  noInterrupts();
  leftTicks = 0;
  rightTicks = 0;
  interrupts();
}

void commandSetSpeed(int value) {
  baseSpeed = constrain(
    value,
    0,
    ABSOLUTE_MAX_MOTOR_PWM
  );
}

void commandSetPid(String payload) {
  payload.trim();

  int comma1 = payload.indexOf(',');
  int comma2 = payload.indexOf(',', comma1 + 1);

  if (comma1 < 0 || comma2 < 0) {
    return;
  }

  String kpText = payload.substring(0, comma1);
  String kiText = payload.substring(comma1 + 1, comma2);
  String kdText = payload.substring(comma2 + 1);

  Kp = kpText.toFloat();
  Ki = kiText.toFloat();
  Kd = kdText.toFloat();

  resetPID();
}

void commandSetBranch(String payload) {
  payload.trim();
  payload.toUpperCase();

  BranchMode requestedMode = BRANCH_AUTO;

  if (payload == "LEFT") {
    requestedMode = BRANCH_LEFT;
  } else if (payload == "RIGHT") {
    requestedMode = BRANCH_RIGHT;
  } else if (payload == "STRAIGHT") {
    requestedMode = BRANCH_STRAIGHT;
  } else if (payload == "AUTO") {
    requestedMode = BRANCH_AUTO;

    if (branchMode == requestedMode) {
      return;
    }

    branchMode = requestedMode;
    branchModeSetTimeMs = 0;
    resetPID();
    return;
  } else {
    return;
  }

  // Refresh the command lifetime but do not repeatedly reset the PID when the
  // same Pi/RFID branch command is received again.
  if (branchMode == requestedMode) {
    branchModeSetTimeMs = millis();
    return;
  }

  branchMode = requestedMode;
  branchModeSetTimeMs = millis();
  resetPID();
}

void commandClearBranch() {
  // CLEAR_BRANCH is an explicit supervisory cancellation of any pending
  // junction context. It must also disarm a passive derailment resume.
  if (
    driveState == STATE_DERAIL_HOLD ||
    driveState == STATE_DERAIL_READY
  ) {
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
  resetPID();
}

void commandPivotLeft(int pwm) {
  if (eStopActive) {
    return;
  }

  pwm = constrain(
    pwm,
    0,
    ABSOLUTE_MAX_MOTOR_PWM
  );

  rawLeftCommand = 0;
  rawRightCommand = 0;

  manualPivotPwm = pwm;
  followEnabled = false;
  driveState = STATE_MANUAL_PIVOT_LEFT;

  lineLostFrameCount = 0;
  reacquireFrameCount = 0;

  clearRecoveryHint();
  commandClearBranch();
  resetPID();

  setDriveCommand(-manualPivotPwm, manualPivotPwm);

  digitalWrite(STATUS_LED, HIGH);
}

void commandPivotRight(int pwm) {
  if (eStopActive) {
    return;
  }

  pwm = constrain(
    pwm,
    0,
    ABSOLUTE_MAX_MOTOR_PWM
  );

  rawLeftCommand = 0;
  rawRightCommand = 0;

  manualPivotPwm = pwm;
  followEnabled = false;
  driveState = STATE_MANUAL_PIVOT_RIGHT;

  lineLostFrameCount = 0;
  reacquireFrameCount = 0;

  clearRecoveryHint();
  commandClearBranch();
  resetPID();

  setDriveCommand(manualPivotPwm, -manualPivotPwm);

  digitalWrite(STATUS_LED, HIGH);
}

void commandRawDrive(String payload) {
  if (eStopActive) {
    return;
  }

  payload.trim();

  int comma1 = payload.indexOf(',');

  if (comma1 < 0) {
    return;
  }

  int leftValue = payload.substring(0, comma1).toInt();
  int rightValue = payload.substring(comma1 + 1).toInt();

  leftValue = constrain(
    leftValue,
    -ABSOLUTE_MAX_MOTOR_PWM,
    ABSOLUTE_MAX_MOTOR_PWM
  );

  rightValue = constrain(
    rightValue,
    -ABSOLUTE_MAX_MOTOR_PWM,
    ABSOLUTE_MAX_MOTOR_PWM
  );

  rawLeftCommand = leftValue;
  rawRightCommand = rightValue;

  manualPivotPwm = 0;
  followEnabled = false;
  driveState = STATE_RAW_DRIVE;

  lineLostFrameCount = 0;
  reacquireFrameCount = 0;

  clearRecoveryHint();
  commandClearBranch();
  resetPID();

  setDriveCommand(rawLeftCommand, rawRightCommand);

  digitalWrite(STATUS_LED, HIGH);
}

// ==================================================
// Status reporting
// ==================================================

void reportStatus() {
  long leftCopy = 0;
  long rightCopy = 0;

  noInterrupts();
  leftCopy = leftTicks;
  rightCopy = rightTicks;
  interrupts();

  int faultCode = getFaultCode();

  unsigned long hintAge = 0;
  int reportHint = 0;

  if (recoveryHintDirection != 0 && recoveryHintTimeMs > 0) {
    hintAge = millis() - recoveryHintTimeMs;

    if (hintAge <= RECOVERY_HINT_MAX_AGE_MS) {
      reportHint = recoveryHintDirection;
    }
  }

  // This must be one uninterrupted newline-terminated status message.  The
  // existing Pi bridge uses readline() with a short timeout and rejects a
  // partially transmitted frame before it reaches FAULT.
  Serial.print("A:STATE=");
  Serial.print(stateToText(driveState));

  Serial.print(";POS=");
  Serial.print(currentLinePosition);
  Serial.print(";L=");
  Serial.print(leftCopy);
  Serial.print(";R=");
  Serial.print(rightCopy);
  Serial.print(";ACTIVE=");
  Serial.print(activeSensorCount);
  Serial.print(";VALID=");
  Serial.print(validLineForTracking ? 1 : 0);
  Serial.print(";LOST=");
  Serial.print(lineLostFrameCount);
  Serial.print(";SPEED=");
  Serial.print(baseSpeed);
  Serial.print(";MAXPWM=");
  Serial.print(ABSOLUTE_MAX_MOTOR_PWM);
  Serial.print(";APPLIEDL=");
  Serial.print(lastLeftCommand);
  Serial.print(";APPLIEDR=");
  Serial.print(lastRightCommand);
  Serial.print(";PIVOT=");
  Serial.print(manualPivotPwm);
  Serial.print(";RAWL=");
  Serial.print(rawLeftCommand);
  Serial.print(";RAWR=");
  Serial.print(rawRightCommand);
  // Follow-only wheel-speed telemetry:
  // DL / DR = requested-direction speed error in mm/s.
  // WCORR = left PWM correction minus right PWM correction.
  Serial.print(";WSYNC=");
  Serial.print(WHEEL_SPEED_CONTROL_ENABLED ? 1 : 0);
  Serial.print(";DL=");
  Serial.print((long)leftSpeedErrorMmS);
  Serial.print(";DR=");
  Serial.print((long)rightSpeedErrorMmS);
  Serial.print(";WCORR=");
  Serial.print((long)(leftWheelPwmCorrection - rightWheelPwmCorrection));
  Serial.print(";ESTOP=");
  Serial.print(eStopActive ? 1 : 0);
  Serial.print(";FAULT=");
  Serial.print(faultCode);
  Serial.print(";HINT=");
  Serial.print(reportHint);
  Serial.print(";HAGE=");
  Serial.print(hintAge);
  Serial.print(";CAND=");
  Serial.print(recoveryCandidateDirection);
  Serial.print(";CFR=");
  Serial.print(recoveryCandidateFrames);
  Serial.print(";CLUST=");
  Serial.print(clusterCount);
  Serial.print(";JUNC=");
  Serial.print(junctionCandidate ? 1 : 0);
  Serial.print(";SEL=");
  Serial.print(selectedClusterIndex);
  Serial.print(";SSTART=");
  Serial.print(selectedClusterStart);
  Serial.print(";SEND=");
  Serial.print(selectedClusterEnd);
  Serial.print(";SPOS=");
  Serial.print(selectedClusterPosition);
  Serial.print(";SACT=");
  Serial.print(selectedClusterActiveCount);
  Serial.print(";BMODE=");
  Serial.print(branchModeToText(branchMode));

  if (REPORT_SENSOR_SIGNALS) {
    Serial.print(";SIG=");

    for (int i = 0; i < SENSOR_COUNT; i++) {
      if (i > 0) {
        Serial.print(',');
      }

      Serial.print(signalValues[i]);
    }
  }

  Serial.println();
}

const char* branchModeToText(int mode) {
  switch (mode) {
    case BRANCH_STRAIGHT:
      return "STRAIGHT";

    case BRANCH_LEFT:
      return "LEFT";

    case BRANCH_RIGHT:
      return "RIGHT";

    case BRANCH_AUTO:
    default:
      return "AUTO";
  }
}

const char* stateToText(int state) {
  switch (state) {
    case STATE_IDLE:
      return "IDLE";

    case STATE_FOLLOW:
      return "FOLLOW";

    case STATE_RECOVER_LEFT:
      return "RECOVER_LEFT";

    case STATE_RECOVER_RIGHT:
      return "RECOVER_RIGHT";

    case STATE_RECOVER_FORWARD:
      return "RECOVER_FORWARD";

    case STATE_DERAIL_HOLD:
      return "DERAIL_HOLD";

    case STATE_DERAIL_READY:
      return "DERAIL_READY";

    case STATE_MANUAL_PIVOT_LEFT:
      return "PIVOT_LEFT";

    case STATE_MANUAL_PIVOT_RIGHT:
      return "PIVOT_RIGHT";

    case STATE_RAW_DRIVE:
      return "RAW_DRIVE";

    case STATE_STOPPED:
      return "STOPPED";

    case STATE_ESTOP:
      return "ESTOP";

    default:
      return "UNKNOWN";
  }
}

int getFaultCode() {
  if (eStopActive || driveState == STATE_ESTOP) {
    return 2;
  }

  if (driveState == STATE_STOPPED) {
    return 1;
  }

  return 0;
}
