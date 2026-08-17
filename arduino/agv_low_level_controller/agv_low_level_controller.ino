/*
  AGV1 LARGE-CHASSIS MIGRATION - STAGE 1
  Arduino Mega Line Follower + Motor Controller

  Hardware:
  - Arduino Mega
  - 13-channel switching IR array on A0-A12
  - IR emitter control on D8
  - Left IBT-2/BTS7960 driver:  RPWM D6,  LPWM D5
  - Right IBT-2/BTS7960 driver: RPWM D11, LPWM D10
  - Quadrature encoders:
      Left  A/B = D2 / D3
      Right A/B = D18 / D19

  Migration guarantees:
  - Preserves AGV1's existing Raspberry Pi command/status protocol.
  - Reports raw signed x4 quadrature ticks; no encoder scaling is done here.
  - Keeps the BMI160/precise-turn logic on the Raspberry Pi unchanged.
  - All IR thresholds are compile-time constants changed in this file.
  - Adds a conservative PWM ceiling and output slew limiting for bench tests.
  - Keeps encoder ticks available for Raspberry Pi odometry and motion control.

  IMPORTANT:
  - IBT-2 R_EN and L_EN must be held HIGH; all grounds must be common.
  - Start with the drive wheels raised and a physical emergency stop available.
  - The command watchdog is disabled until the Raspberry Pi sends a heartbeat.

  Behavior:
  - Normal line following with PID
  - Dynamic sharp-bend recovery based on recent POS direction
  - No permanent old turn memory
  - Manual pivot commands for ROS2 IMU turn manager
  - RAW_DRIVE command for short encoder-based straight movement from marker to junction center
  - Marker reporting:
      ACTIVE   = dynamic line-tracking active count
      MACTIVE  = separate marker active count
      MTH      = marker signal threshold
      WIDE=1   when many marker-active sensors see white
      SOLID=1  when wide/solid white is held for more frames
      MARKER=1 when WIDE=1
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

// Sensor layout:
// A0 = rightmost
// A12 = leftmost
// Positive position = line toward right side
// Negative position = line toward left side
// The +/-350 range intentionally matches the old 8-sensor AGV1 firmware,
// so the first PID tests can begin with the old numerical error scale.
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
//const int MIN_VALID_OFF_VALUE = 100;

// Per-sensor reflected-signal thresholds used for line tracking.
// signalValues[i] = IR-off reading - IR-on reading.
// Replace these values after recording SIG over the line and floor.
// Set REPORT_SENSOR_SIGNALS to true temporarily during calibration.
const int SENSOR_SIGNAL_THRESHOLD[SENSOR_COUNT] = {
  /* A0  rightmost */ 200,
  /* A1            */ 200,
  /* A2            */ 200,
  /* A3            */ 200,
  /* A4            */ 200,
  /* A5            */ 200,
  /* A6  centre    */ 200,
  /* A7            */ 200,
  /* A8            */ 200,
  /* A9            */ 200,
  /* A10           */ 200,
  /* A11           */ 200,
  /* A12 leftmost  */ 200
};

const bool REPORT_SENSOR_SIGNALS = true;

const int ANALOG_SAMPLES = 3;
const int ANALOG_SAMPLE_DELAY_US = 100;
const int IR_SETTLE_DELAY_US = 1000;

// ==================================================
// PID settings
// ==================================================

// Initial values only. The weight range matches the former AGV1 firmware,
// but the larger chassis must be tuned at low speed.
float Kp = 0.35;
float Ki = 0.00;
float Kd = 0.70;

int baseSpeed = 30;

// Absolute safety ceiling applied throughout the complete motor command path.
// 40/255 = 15.7% duty cycle.
const int ABSOLUTE_MAX_MOTOR_PWM = 40;

// Line-following limits. The absolute ceiling below remains authoritative.
const int MAX_NORMAL_LINE_STEERING = 65;
const int MAX_CORNER_LINE_STEERING = 120;
const int MAX_CORNER_REVERSE_PWM = 60;
const int MAX_DERIVATIVE_STEP = 180;

// Applied once per 20 ms control cycle. Commands reverse through zero.
const int PWM_RISE_PER_CONTROL = 3;
const int PWM_FALL_PER_CONTROL = 6;

const float INTEGRAL_LIMIT = 300.0;

// ==================================================
// Line recovery settings
// ==================================================

const int LINE_LOST_RECOVERY_FRAMES = 2;
const int LINE_LOST_STOP_FRAMES = 25;

const int TURN_RECOVERY_PWM = 30;

const int CORNER_DETECT_POSITION = 250;

const int REACQUIRE_POSITION_TOLERANCE = 150;
const int REACQUIRE_CONFIRM_FRAMES = 3;
const unsigned long MAX_TURN_RECOVERY_TIME_MS = 3000;

// Dynamic recovery hint.
// Recent POS positive -> recover right.
// Recent POS negative -> recover left.
const int RECOVERY_DIRECTION_MIN_POSITION = 35;
const int RECOVERY_DIRECTION_CONFIRM_FRAMES = 2;
const unsigned long RECOVERY_HINT_MAX_AGE_MS = 700;

// ==================================================
// Marker / feature detection settings
// ==================================================

// const int MARKER_SIGNAL_MIN = 60;
// const int MARKER_SIGNAL_PERCENT_OF_MAX = 45;

// Old AGV1 used 6/8 for WIDE and 7/8 for SOLID.
// These are the nearest equivalent proportions for 13 sensors.
const int WIDE_ACTIVE_MIN = 10;
const int SOLID_ACTIVE_MIN = 12;

const int WIDE_CONFIRM_FRAMES = 3;
const int SOLID_CONFIRM_FRAMES = 5;

// ==================================================
// Timing
// ==================================================

const unsigned long CONTROL_INTERVAL_MS = 20;
const unsigned long STATUS_INTERVAL_MS = 50;

// Keep disabled for protocol compatibility: one C:START currently permits
// continuous following, and the existing Pi interface has no heartbeat yet.
// Enable only after a ROS serial node sends any valid command more frequently
// than COMMAND_WATCHDOG_TIMEOUT_MS.
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
  STATE_MANUAL_PIVOT_LEFT,
  STATE_MANUAL_PIVOT_RIGHT,
  STATE_RAW_DRIVE,
  STATE_STOPPED,
  STATE_ESTOP
};

DriveState driveState = STATE_IDLE;

bool followEnabled = false;
bool eStopActive = false;

// ==================================================
// Sensor state
// ==================================================

int offValues[SENSOR_COUNT];
int onValues[SENSOR_COUNT];
int signalValues[SENSOR_COUNT];

int currentLinePosition = 0;
int activeSensorCount = 0;
bool validLineForTracking = false;

int lineLostFrameCount = 0;
int reacquireFrameCount = 0;

// ==================================================
// Dynamic recovery hint state
// ==================================================

int recoveryHintDirection = 0;
unsigned long recoveryHintTimeMs = 0;

int recoveryCandidateDirection = 0;
int recoveryCandidateFrames = 0;

// ==================================================
// Marker / feature state
// ==================================================

bool wideFeatureDetected = false;
bool solidFeatureDetected = false;
bool markerDetected = false;

int wideFrameCount = 0;
int solidFrameCount = 0;

int markerActiveSensorCount = 0;
int markerSignalThreshold = 0;

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
      offSums[i] += analogRead(SENSOR_PINS[i]);
    }
    delayMicroseconds(ANALOG_SAMPLE_DELAY_US);
  }

  setIrEmitter(true);
  delayMicroseconds(IR_SETTLE_DELAY_US);

  for (int sample = 0; sample < ANALOG_SAMPLES; sample++) {
    for (int i = 0; i < SENSOR_COUNT; i++) {
      onSums[i] += analogRead(SENSOR_PINS[i]);
    }
    delayMicroseconds(ANALOG_SAMPLE_DELAY_US);
  }

  setIrEmitter(false);

  int maxOffValue = 0;
  int maxSignal = 0;
  int minSignal = 1023;

  for (int i = 0; i < SENSOR_COUNT; i++) {
    offValues[i] = offSums[i] / ANALOG_SAMPLES;
    onValues[i] = onSums[i] / ANALOG_SAMPLES;

    int signal = onValues[i] - offValues[i];

    if (signal < 0) {
      signal = 0;
    }

    signalValues[i] = signal;

    if (offValues[i] > maxOffValue) {
      maxOffValue = offValues[i];
    }

    if (signalValues[i] > maxSignal) {
      maxSignal = signalValues[i];
    }

    if (signalValues[i] < minSignal) {
      minSignal = signalValues[i];
    }
  }

  activeSensorCount = 0;
  validLineForTracking = false;

  //if (maxOffValue < MIN_VALID_OFF_VALUE) {
  //  return;
  //}

  if (maxSignal < MIN_MAX_SIGNAL) {
    return;
  }

  int contrast = maxSignal - minSignal;

  if (contrast < MIN_CONTRAST) {
    return;
  }

  long weightedSum = 0;
  long totalSignal = 0;

  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (signalValues[i] >= SENSOR_SIGNAL_THRESHOLD[i]) {
      activeSensorCount++;
      weightedSum += (long)signalValues[i] * SENSOR_WEIGHTS[i];
      totalSignal += signalValues[i];
    }
  }

  if (activeSensorCount <= 0 || totalSignal <= 0) {
    return;
  }

  currentLinePosition = weightedSum / totalSignal;
  validLineForTracking = true;
}

// ==================================================
// Marker / feature detection
// ==================================================

void updateMarkerDetection() {
  markerActiveSensorCount = 0;

  // Count sensors using the same calibrated thresholds
  // used by the line detector.
  for (int i = 0; i < SENSOR_COUNT; i++) {
    if (signalValues[i] >= SENSOR_SIGNAL_THRESHOLD[i]) {
      markerActiveSensorCount++;
    }
  }

  bool wideCandidate =
      markerActiveSensorCount >= WIDE_ACTIVE_MIN;

  bool solidCandidate =
      markerActiveSensorCount >= SOLID_ACTIVE_MIN;

  if (wideCandidate) {
    if (wideFrameCount < WIDE_CONFIRM_FRAMES) {
      wideFrameCount++;
    }
  } else {
    wideFrameCount = 0;
  }

  if (solidCandidate) {
    if (solidFrameCount < SOLID_CONFIRM_FRAMES) {
      solidFrameCount++;
    }
  } else {
    solidFrameCount = 0;
  }

  wideFeatureDetected =
      wideFrameCount >= WIDE_CONFIRM_FRAMES;

  solidFeatureDetected =
      solidFrameCount >= SOLID_CONFIRM_FRAMES;

  markerDetected = wideFeatureDetected;

  // Only retained for telemetry because all sensor
  // thresholds are presently 200.
  markerSignalThreshold = 200;
}

// ==================================================
// Main control loop
// ==================================================

void runControlLoop() {
  readSensorsSwitching();
  updateMarkerDetection();

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

  if (driveState == STATE_RECOVER_LEFT || driveState == STATE_RECOVER_RIGHT) {
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

  int freshHint = getFreshRecoveryHint();

  if (freshHint > 0) {
    startRecoveryRight();
    handleRecoveryState();
    return;
  }

  if (freshHint < 0) {
    startRecoveryLeft();
    handleRecoveryState();
    return;
  }

  stopDueToNoFreshRecoveryHint();
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

  // Independent PID-output safety clamp.
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

  setDriveCommand(leftCommand, rightCommand);
}

void resetPID() {
  pidIntegral = 0.0;
  lastError = 0.0;
}

// ==================================================
// Dynamic recovery hint logic
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

void startRecoveryLeft() {
  driveState = STATE_RECOVER_LEFT;
  turnRecoveryStartTime = millis();
  reacquireFrameCount = 0;
  resetPID();
}

void startRecoveryRight() {
  driveState = STATE_RECOVER_RIGHT;
  turnRecoveryStartTime = millis();
  reacquireFrameCount = 0;
  resetPID();
}

void handleRecoveryState() {
  if (validLineForTracking) {
    if (abs(currentLinePosition) <= REACQUIRE_POSITION_TOLERANCE) {
      reacquireFrameCount++;

      if (reacquireFrameCount >= REACQUIRE_CONFIRM_FRAMES) {
        lineLostFrameCount = 0;
        reacquireFrameCount = 0;

        clearRecoveryHint();

        driveState = STATE_FOLLOW;
        resetPID();
        applyLinePid();
        return;
      }
    } else {
      reacquireFrameCount = 0;
    }
  } else {
    reacquireFrameCount = 0;
  }

  unsigned long elapsed = millis() - turnRecoveryStartTime;

  if (elapsed > MAX_TURN_RECOVERY_TIME_MS) {
    followEnabled = false;
    driveState = STATE_STOPPED;
    stopMotors();
    return;
  }

  if (driveState == STATE_RECOVER_LEFT) {
    setDriveCommand(-TURN_RECOVERY_PWM, TURN_RECOVERY_PWM);
  } else if (driveState == STATE_RECOVER_RIGHT) {
    setDriveCommand(TURN_RECOVERY_PWM, -TURN_RECOVERY_PWM);
  }
}

void stopDueToNoFreshRecoveryHint() {
  followEnabled = false;
  driveState = STATE_STOPPED;

  lineLostFrameCount = 0;
  reacquireFrameCount = 0;

  clearRecoveryHint();
  resetPID();
  stopMotors();
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

  // A requested direction reversal must pass through zero first.
  if ((applied > 0 && target < 0) || (applied < 0 && target > 0)) {
    target = 0;
  }

  int step = PWM_RISE_PER_CONTROL;

  // Moving toward zero is deceleration and may use the faster step.
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

void stopMotors() {
  lastLeftCommand = 0;
  lastRightCommand = 0;

  rawLeftCommand = 0;
  rawRightCommand = 0;

  analogWrite(LEFT_RPWM_PIN, 0);
  analogWrite(LEFT_LPWM_PIN, 0);
  analogWrite(RIGHT_RPWM_PIN, 0);
  analogWrite(RIGHT_LPWM_PIN, 0);
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
  if (eStopActive) {
    return;
  }

  rawLeftCommand = 0;
  rawRightCommand = 0;

  followEnabled = true;
  driveState = STATE_FOLLOW;

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

  clearRecoveryHint();
  resetPID();
  stopMotors();

  digitalWrite(STATUS_LED, LOW);
}

void commandEstop() {
  eStopActive = true;
  followEnabled = false;
  driveState = STATE_ESTOP;

  rawLeftCommand = 0;
  rawRightCommand = 0;
  manualPivotPwm = 0;

  lineLostFrameCount = 0;
  reacquireFrameCount = 0;

  clearRecoveryHint();
  resetPID();
  stopMotors();

  digitalWrite(STATUS_LED, LOW);
}

void commandReset() {
  eStopActive = false;
  followEnabled = false;
  driveState = STATE_IDLE;

  rawLeftCommand = 0;
  rawRightCommand = 0;
  manualPivotPwm = 0;

  lineLostFrameCount = 0;
  reacquireFrameCount = 0;

  clearRecoveryHint();
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

  // Retained as fixed values for compatibility with Raspberry Pi parsers
  // written for the former wheel-synchronization telemetry format.
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

  Serial.print(";MACTIVE=");
  Serial.print(markerActiveSensorCount);

  Serial.print(";MTH=");
  Serial.print(markerSignalThreshold);

  Serial.print(";WIDE=");
  Serial.print(wideFeatureDetected ? 1 : 0);

  Serial.print(";SOLID=");
  Serial.print(solidFeatureDetected ? 1 : 0);

  Serial.print(";MARKER=");
  Serial.print(markerDetected ? 1 : 0);

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

const char* stateToText(DriveState state) {
  switch (state) {
    case STATE_IDLE:
      return "IDLE";

    case STATE_FOLLOW:
      return "FOLLOW";

    case STATE_RECOVER_LEFT:
      return "RECOVER_LEFT";

    case STATE_RECOVER_RIGHT:
      return "RECOVER_RIGHT";

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
