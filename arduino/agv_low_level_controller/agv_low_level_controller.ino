/*
  AGV1 LARGE-CHASSIS MIGRATION - 13-SENSOR WHITE LINE FOLLOWER

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

// ==================================================
// Hardware calibration
// ==================================================

const bool IR_CONTROL_ACTIVE_HIGH = true;

const bool INVERT_LEFT_MOTOR = false;
const bool INVERT_RIGHT_MOTOR = true;

const bool INVERT_STEERING = false;

const bool INVERT_LEFT_ENCODER = true;
const bool INVERT_RIGHT_ENCODER = true;

// ==================================================
// IR detection settings
// ==================================================

const int MIN_MAX_SIGNAL = 40;
const int MIN_CONTRAST = 20;

// White-line detection:
// signalValues[i] >= SENSOR_SIGNAL_THRESHOLD[i]
const int SENSOR_SIGNAL_THRESHOLD[SENSOR_COUNT] = {
  /* A0  rightmost */ 245,
  /* A1            */ 255,
  /* A2            */ 350,
  /* A3            */ 310,
  /* A4            */ 320,
  /* A5            */ 320,
  /* A6  centre    */ 300,
  /* A7            */ 355,
  /* A8            */ 315,
  /* A9            */ 315,
  /* A10           */ 290,
  /* A11           */ 265,
  /* A12 leftmost  */ 190
};

const int LINE_ACTIVE_MIN = 1;
const int LINE_TOTAL_STRENGTH_MIN = 20;

const bool REPORT_SENSOR_SIGNALS = true;

const int ANALOG_SAMPLES = 5;
const int ANALOG_SAMPLE_DELAY_US = 100;
const int IR_SETTLE_DELAY_US = 1000;

// ==================================================
// PID settings
// ==================================================

float Kp = 0.11;
float Ki = 0.02;
float Kd = 1.3;

int baseSpeed = 40;

const int ABSOLUTE_MAX_MOTOR_PWM = 40;

const int MAX_NORMAL_LINE_STEERING = 30;
const int MAX_CORNER_LINE_STEERING = 40;
const int MAX_CORNER_REVERSE_PWM = 0;
const int MAX_DERIVATIVE_STEP = 180;

const int PWM_RISE_PER_CONTROL = 3;
const int PWM_FALL_PER_CONTROL = 6;

const float INTEGRAL_LIMIT = 300.0;

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
const int LINE_LOST_STOP_FRAMES = 40;

const int FORWARD_RECOVERY_PWM = 18;
const unsigned long FORWARD_RECOVERY_TIME_MS = 350;

const int CORNER_DETECT_POSITION = 220;

const int REACQUIRE_POSITION_TOLERANCE = 170;
const int REACQUIRE_CONFIRM_FRAMES = 5;
const unsigned long MAX_TURN_RECOVERY_TIME_MS = 2500;

const int RECOVERY_DIRECTION_MIN_POSITION = 40;
const int RECOVERY_DIRECTION_CONFIRM_FRAMES = 3;
const unsigned long RECOVERY_HINT_MAX_AGE_MS = 800;

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

const int STRAIGHT_BRANCH_FIRST_INDEX = 4;
const int STRAIGHT_BRANCH_LAST_INDEX = 8;

// ==================================================
// Timing
// ==================================================

const unsigned long CONTROL_INTERVAL_MS = 20;
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
  if (branchMode == BRANCH_AUTO) {
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
  }

  if (selected) {
    return;
  }

  BranchMode fallbackMode = BRANCH_AUTO;

  if (branchMode == BRANCH_LEFT || branchMode == BRANCH_RIGHT) {
    fallbackMode = branchMode;
  } else if (branchMode == BRANCH_STRAIGHT || junctionCandidate) {
    fallbackMode = BRANCH_STRAIGHT;
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

  clearLineSelectionState();
  expireBranchCommandIfNeeded();

  if (maxSignal < MIN_MAX_SIGNAL) {
    return;
  }

  int contrast = maxSignal - minSignal;

  if (contrast < MIN_CONTRAST) {
    return;
  }

  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (signalValues[i] >= SENSOR_SIGNAL_THRESHOLD[i]) {
      lineStrengthValues[i] = signalValues[i] - SENSOR_SIGNAL_THRESHOLD[i];
      lineActiveMask[i] = true;
    }
  }

  selectLineForTracking();
}

void runControlLoop() {
  readSensorsSwitching();

  if (eStopActive) {
    stopMotors();
    driveState = STATE_ESTOP;
    return;
  }

  if (driveState == STATE_RAW_DRIVE) {
    setDriveCommand(rawLeftCommand, rawRightCommand);
    return;
  }

  if (driveState == STATE_MANUAL_PIVOT_LEFT) {
    setDriveCommand(-manualPivotPwm, manualPivotPwm);
    return;
  }

  if (driveState == STATE_MANUAL_PIVOT_RIGHT) {
    setDriveCommand(manualPivotPwm, -manualPivotPwm);
    return;
  }

  if (!followEnabled) {
    if (driveState != STATE_STOPPED) {
      driveState = STATE_IDLE;
    }

    stopMotors();
    return;
  }

  if (driveState == STATE_RECOVER_FORWARD) {
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

  lineLostFrameCount++;

  if (lineLostFrameCount < LINE_LOST_RECOVERY_FRAMES) {
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
  float error = (float)currentLinePosition;

  if (INVERT_STEERING) {
    error = -error;
  }

  pidIntegral += error;

  if (pidIntegral > INTEGRAL_LIMIT) {
    pidIntegral = INTEGRAL_LIMIT;
  } else if (pidIntegral < -INTEGRAL_LIMIT) {
    pidIntegral = -INTEGRAL_LIMIT;
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

  steering = constrain(
    steering,
    -steeringLimit,
    steeringLimit
  );

  int leftCommand = baseSpeed + (int)steering;
  int rightCommand = baseSpeed - (int)steering;

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

  setDriveCommand(leftCommand, rightCommand);
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

void handleRecoveryState() {
  unsigned long elapsed = millis() - turnRecoveryStartTime;

  if (elapsed < FORWARD_RECOVERY_TIME_MS) {
    setDriveCommand(FORWARD_RECOVERY_PWM, FORWARD_RECOVERY_PWM);
    return;
  }

  followEnabled = false;
  driveState = STATE_STOPPED;
  recoveryStopLatched = true;

  lineLostFrameCount = 0;
  reacquireFrameCount = 0;

  clearRecoveryHint();
  commandClearBranch();
  resetPID();
  brakeAndStopMotors();
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
}

void applyBrakePulseToMotor(int rpwmPin, int lpwmPin) {
  analogWrite(rpwmPin, ACTIVE_BRAKE_PWM);
  analogWrite(lpwmPin, ACTIVE_BRAKE_PWM);
}

void brakeMotorOutputsBriefly() {
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
  followEnabled = false;
  driveState = STATE_STOPPED;
  recoveryStopLatched = true;

  lineLostFrameCount = 0;
  reacquireFrameCount = 0;

  clearRecoveryHint();
  commandClearBranch();
  resetPID();
  brakeAndStopMotors();
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
    commandStart();
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

  if (payload == "LEFT") {
    branchMode = BRANCH_LEFT;
  } else if (payload == "RIGHT") {
    branchMode = BRANCH_RIGHT;
  } else if (payload == "STRAIGHT") {
    branchMode = BRANCH_STRAIGHT;
  } else if (payload == "AUTO") {
    branchMode = BRANCH_AUTO;
    branchModeSetTimeMs = 0;
    return;
  } else {
    return;
  }

  branchModeSetTimeMs = millis();
}

void commandClearBranch() {
  branchMode = BRANCH_AUTO;
  branchModeSetTimeMs = 0;
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

  Serial.print(";WSYNC=0");
  Serial.print(";DL=0");
  Serial.print(";DR=0");
  Serial.print(";WCORR=0");

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
