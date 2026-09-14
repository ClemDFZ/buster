#include <AccelStepper.h>
#include <string.h>

#include "cobs.h"
#include "crc16.h"
#include "frame.h"

/*
 * pantilt_state_machine - DRV8825 + AccelStepper with IDLE/HOMING/RUNNING
 *
 * Pins:
 *   PAN  : DIR=26 STEP=27 EN=25
 *   TILT : DIR=17 STEP=18 EN=16
 *   KY-008 laser: GPIO23
 *   TILT endstop: GPIO19 (NO wiring, INPUT_PULLUP — LOW=triggered)
 *
 * Serial (921600): COBS-framed binary PtCmdFrame / PtStatusFrame + 0x00 delimiter
 */

static const uint8_t PAN_DIR_PIN = 26;
static const uint8_t PAN_STEP_PIN = 27;
static const uint8_t PAN_EN_PIN = 25;
static const uint8_t TILT_DIR_PIN = 17;
static const uint8_t TILT_STEP_PIN = 18;
static const uint8_t TILT_EN_PIN = 16;
static const uint8_t LASER_PIN = 23;
static const uint8_t TILT_ENDSTOP_PIN = 19;

static const float MOTOR_STEPS_PER_REV = 200.0f;
static const float MICROSTEPS = 16.0f;
static const float GEAR_RATIO_PAN = 1.0f;
static const float GEAR_RATIO_TILT = 2.5f;
static const float MAX_SPEED_STEPS_S = 15000.0f;
static const float ACCEL_STEPS_S2 = 25000.0f;
static const bool INVERT_PAN = false;
static const bool INVERT_TILT = false;

static const float PAN_STEPS_PER_DEG = (MOTOR_STEPS_PER_REV * MICROSTEPS * GEAR_RATIO_PAN) / 360.0f;
static const float TILT_STEPS_PER_DEG = (MOTOR_STEPS_PER_REV * MICROSTEPS * GEAR_RATIO_TILT) / 360.0f;

static const unsigned long CMD_IDLE_TIMEOUT_MS = 180000UL;
static const unsigned long ENDSTOP_DEBOUNCE_MS = 40UL;
static const float HOMING_TILT_DPS = 20.0f;
static const float HOMING_SPEED_STEPS_S = HOMING_TILT_DPS * TILT_STEPS_PER_DEG;
static const long FAR_TARGET = 999999L;

static const float PAN_MIN_DEG = -180.0f;
static const float PAN_MAX_DEG = 180.0f;
static const float TILT_MIN_DEG = -75.0f;       // travel limit
static const float TILT_MAX_DEG = 0.0f;         // endstop / home
static const float HOMING_PARK_TILT_DEG = -45.0f;  // pose after homing
static const float LIMIT_EPS_DEG = 0.05f;

AccelStepper panStepper(AccelStepper::DRIVER, PAN_STEP_PIN, PAN_DIR_PIN);
AccelStepper tiltStepper(AccelStepper::DRIVER, TILT_STEP_PIN, TILT_DIR_PIN);

enum State : uint8_t {
  ST_IDLE,
  ST_HOMING,
  ST_RUNNING
};

enum HomingPhase : uint8_t {
  HM_BACKOFF,  // leave switch (until released, not to TILT_MIN)
  HM_SEEK,     // drive into switch
  HM_PARK      // after zero: go to HOMING_PARK_TILT_DEG
};

static State currentState = ST_IDLE;
static HomingPhase homingPhase = HM_SEEK;
static bool homingSawRelease = false;
static bool haveHomed = false;
static unsigned long homingDebounceMs = 0;
static bool panEnabled = false;
static bool tiltEnabled = false;
static unsigned long lastAnyCmdMs = 0;
static unsigned long lastPanCmdMs = 0;
static unsigned long lastTiltCmdMs = 0;
static uint32_t txSeq = 0;

static uint8_t rxBuf[128];
static size_t rxLen = 0;

static uint8_t stateToMode(State s) {
  switch (s) {
    case ST_IDLE: return PT_MODE_IDLE;
    case ST_HOMING: return PT_MODE_HOMING;
    case ST_RUNNING: return PT_MODE_RUNNING;
  }
  return PT_MODE_IDLE;
}

static void markAnyCmd() { lastAnyCmdMs = millis(); }
static void markPanCmd() {
  markAnyCmd();
  lastPanCmdMs = millis();
}
static void markTiltCmd() {
  markAnyCmd();
  lastTiltCmdMs = millis();
}

static void updateLaser() {
  if (currentState != ST_RUNNING) {
    digitalWrite(LASER_PIN, LOW);
    return;
  }
  unsigned long now = millis();
  const bool panRecent =
      lastPanCmdMs != 0 && (now - lastPanCmdMs) < CMD_IDLE_TIMEOUT_MS;
  const bool tiltRecent =
      lastTiltCmdMs != 0 && (now - lastTiltCmdMs) < CMD_IDLE_TIMEOUT_MS;
  digitalWrite(LASER_PIN, (panRecent || tiltRecent) ? HIGH : LOW);
}

static void enablePan() {
  if (!panEnabled) {
    digitalWrite(PAN_EN_PIN, LOW);
    panEnabled = true;
  }
}

static void enableTilt() {
  if (!tiltEnabled) {
    digitalWrite(TILT_EN_PIN, LOW);
    tiltEnabled = true;
  }
}

static void disablePan() {
  if (panEnabled) {
    digitalWrite(PAN_EN_PIN, HIGH);
    panEnabled = false;
  }
}

static void disableTilt() {
  if (tiltEnabled) {
    digitalWrite(TILT_EN_PIN, HIGH);
    tiltEnabled = false;
  }
}

static void disableAllMotors() {
  disablePan();
  disableTilt();
}

static long degToPanSteps(float deg) {
  float sign = INVERT_PAN ? -1.0f : 1.0f;
  return lroundf(sign * deg * PAN_STEPS_PER_DEG);
}

static long degToTiltSteps(float deg) {
  float sign = INVERT_TILT ? -1.0f : 1.0f;
  return lroundf(sign * deg * TILT_STEPS_PER_DEG);
}

static float panStepsToDeg(long steps) {
  float sign = INVERT_PAN ? -1.0f : 1.0f;
  return (float)steps / (PAN_STEPS_PER_DEG * sign);
}

static float tiltStepsToDeg(long steps) {
  float sign = INVERT_TILT ? -1.0f : 1.0f;
  return (float)steps / (TILT_STEPS_PER_DEG * sign);
}

static float clampPanDeg(float deg) {
  return constrain(deg, PAN_MIN_DEG, PAN_MAX_DEG);
}

static float clampTiltDeg(float deg) {
  return constrain(deg, TILT_MIN_DEG, TILT_MAX_DEG);
}

static long clampPanTargetSteps(long steps) {
  const long minSteps = degToPanSteps(PAN_MIN_DEG);
  const long maxSteps = degToPanSteps(PAN_MAX_DEG);
  if (minSteps <= maxSteps) return constrain(steps, minSteps, maxSteps);
  return constrain(steps, maxSteps, minSteps);
}

static long clampTiltTargetSteps(long steps) {
  const long minSteps = degToTiltSteps(TILT_MIN_DEG);
  const long maxSteps = degToTiltSteps(TILT_MAX_DEG);
  if (minSteps <= maxSteps) return constrain(steps, minSteps, maxSteps);
  return constrain(steps, maxSteps, minSteps);
}

static void enforcePanLimits() {
  const long pos = panStepper.currentPosition();
  const long clamped = clampPanTargetSteps(pos);
  if (pos != clamped) {
    panStepper.setCurrentPosition(clamped);
    panStepper.stop();
    panStepper.moveTo(clamped);
  }
}

static void enforceTiltLimits() {
  const long pos = tiltStepper.currentPosition();
  const long clamped = clampTiltTargetSteps(pos);
  if (pos != clamped) {
    tiltStepper.setCurrentPosition(clamped);
    tiltStepper.stop();
    tiltStepper.moveTo(clamped);
  }
}

static void txWire(const uint8_t *frame, size_t len) {
  uint8_t enc[64];
  const size_t n = cobs_encode(frame, len, enc, sizeof(enc));
  if (n == 0) return;
  Serial.write(enc, n);
  Serial.write(static_cast<uint8_t>(0));
}

static void txStatus(uint8_t op) {
  PtStatusFrame fr;
  memset(&fr, 0, sizeof(fr));
  fr.magic = PT_MAGIC;
  fr.ver = PT_VER;
  fr.op = op;
  fr.mode = stateToMode(currentState);
  fr.seq = txSeq++;
  fr.pan_deg = panStepsToDeg(panStepper.currentPosition());
  fr.tilt_deg = tiltStepsToDeg(tiltStepper.currentPosition());
  fr.target_pan_deg = panStepsToDeg(panStepper.targetPosition());
  fr.target_tilt_deg = tiltStepsToDeg(tiltStepper.targetPosition());
  fr.crc16 = crc16_ccitt_false(reinterpret_cast<const uint8_t *>(&fr), PT_STATUS_CRC_LEN);
  txWire(reinterpret_cast<const uint8_t *>(&fr), PT_STATUS_SIZE);
}

static void announceMode() {
  txStatus(PT_OP_MODE_EVT);
}

static void enterIdle() {
  panStepper.stop();
  tiltStepper.stop();
  disableAllMotors();
  currentState = ST_IDLE;
  digitalWrite(LASER_PIN, LOW);
  announceMode();
}

static void enterRunning() {
  enablePan();
  enableTilt();
  panStepper.setMaxSpeed(MAX_SPEED_STEPS_S);
  panStepper.setAcceleration(ACCEL_STEPS_S2);
  tiltStepper.setMaxSpeed(MAX_SPEED_STEPS_S);
  tiltStepper.setAcceleration(ACCEL_STEPS_S2);
  currentState = ST_RUNNING;
  markAnyCmd();
  announceMode();
}

static bool endstopPressed() {
  return digitalRead(TILT_ENDSTOP_PIN) == LOW;
}

static void startHomingSeek() {
  homingPhase = HM_SEEK;
  homingDebounceMs = 0;
  tiltStepper.setMaxSpeed(HOMING_SPEED_STEPS_S);
  tiltStepper.setAcceleration(ACCEL_STEPS_S2);
  const float sign = INVERT_TILT ? -1.0f : 1.0f;
  tiltStepper.moveTo((long)(sign * FAR_TARGET));
}

static void startHomingBackoff() {
  homingPhase = HM_BACKOFF;
  homingSawRelease = false;
  homingDebounceMs = 0;
  tiltStepper.setCurrentPosition(degToTiltSteps(TILT_MAX_DEG));
  tiltStepper.setMaxSpeed(HOMING_SPEED_STEPS_S);
  tiltStepper.setAcceleration(ACCEL_STEPS_S2);
  // Away from switch; abort on release — do not run to TILT_MIN.
  tiltStepper.moveTo(degToTiltSteps(TILT_MIN_DEG));
}

static void startHomingPark() {
  homingPhase = HM_PARK;
  tiltStepper.stop();
  while (tiltStepper.distanceToGo() != 0) {
    tiltStepper.run();
  }
  tiltStepper.setCurrentPosition(degToTiltSteps(TILT_MAX_DEG));
  haveHomed = true;
  tiltStepper.setMaxSpeed(HOMING_SPEED_STEPS_S);
  tiltStepper.setAcceleration(ACCEL_STEPS_S2);
  tiltStepper.moveTo(degToTiltSteps(HOMING_PARK_TILT_DEG));
}

static void enterHoming() {
  enablePan();
  enableTilt();

  panStepper.stop();
  panStepper.setCurrentPosition(0);

  currentState = ST_HOMING;
  announceMode();

  tiltStepper.stop();
  tiltStepper.setMaxSpeed(HOMING_SPEED_STEPS_S);
  tiltStepper.setAcceleration(ACCEL_STEPS_S2);

  const float tiltNow = tiltStepsToDeg(tiltStepper.currentPosition());
  const bool alreadyHome =
      endstopPressed() || (haveHomed && fabsf(tiltNow - TILT_MAX_DEG) < 3.0f);

  if (alreadyHome) {
    startHomingBackoff();
  } else {
    homingSawRelease = true;
    startHomingSeek();
  }
}

static void tickHoming() {
  if (homingPhase == HM_BACKOFF) {
    if (!endstopPressed()) {
      if (homingDebounceMs == 0) homingDebounceMs = millis();
      if ((millis() - homingDebounceMs) >= ENDSTOP_DEBOUNCE_MS) {
        homingSawRelease = true;
        startHomingSeek();
      }
      return;
    }
    homingDebounceMs = 0;
    return;
  }

  if (homingPhase == HM_SEEK) {
    if (endstopPressed() && homingSawRelease) {
      startHomingPark();
    }
    return;
  }

  // HM_PARK: -45° only after zero. Do not overlap with SEEK target.
  if (tiltStepper.distanceToGo() == 0) {
    enterRunning();
  }
}

static void setPanDeg(float panDeg) {
  markPanCmd();
  enablePan();
  panStepper.moveTo(degToPanSteps(clampPanDeg(panDeg)));
}

static void setTiltDeg(float tiltDeg) {
  markTiltCmd();
  enableTilt();
  tiltStepper.moveTo(degToTiltSteps(clampTiltDeg(tiltDeg)));
}

static void setAnglesDeg(float panDeg, float tiltDeg) {
  markPanCmd();
  markTiltCmd();
  enablePan();
  enableTilt();
  panStepper.moveTo(degToPanSteps(clampPanDeg(panDeg)));
  tiltStepper.moveTo(degToTiltSteps(clampTiltDeg(tiltDeg)));
}

static void moveDeltaDeg(float dpanDeg, float dtiltDeg) {
  if (dpanDeg != 0.0f) {
    markPanCmd();
    enablePan();
  }
  if (dtiltDeg != 0.0f) {
    markTiltCmd();
    enableTilt();
  }
  const float panTarget = clampPanDeg(
      panStepsToDeg(panStepper.targetPosition()) + dpanDeg);
  const float tiltTarget = clampTiltDeg(
      tiltStepsToDeg(tiltStepper.targetPosition()) + dtiltDeg);
  panStepper.moveTo(degToPanSteps(panTarget));
  tiltStepper.moveTo(degToTiltSteps(tiltTarget));
}

static void setVelocity(float panDps, float tiltDps) {
  const float panPos = panStepsToDeg(panStepper.currentPosition());
  const float tiltPos = tiltStepsToDeg(tiltStepper.currentPosition());

  float panSps = fabsf(panDps) * PAN_STEPS_PER_DEG;
  float tiltSps = fabsf(tiltDps) * TILT_STEPS_PER_DEG;

  if (panDps > 0.0f && panPos >= (PAN_MAX_DEG - LIMIT_EPS_DEG)) panSps = 0.0f;
  if (panDps < 0.0f && panPos <= (PAN_MIN_DEG + LIMIT_EPS_DEG)) panSps = 0.0f;
  if (tiltDps > 0.0f && tiltPos >= (TILT_MAX_DEG - LIMIT_EPS_DEG)) tiltSps = 0.0f;
  if (tiltDps < 0.0f && tiltPos <= (TILT_MIN_DEG + LIMIT_EPS_DEG)) tiltSps = 0.0f;

  if (panSps >= 0.5f) markPanCmd();
  if (tiltSps >= 0.5f) markTiltCmd();

  if (panSps < 0.5f) {
    panStepper.stop();
  } else {
    enablePan();
    panStepper.setMaxSpeed(min(panSps, MAX_SPEED_STEPS_S));
    const float limitDeg = (panDps > 0.0f) ? PAN_MAX_DEG : PAN_MIN_DEG;
    panStepper.moveTo(degToPanSteps(limitDeg));
  }

  if (tiltSps < 0.5f) {
    tiltStepper.stop();
  } else {
    enableTilt();
    tiltStepper.setMaxSpeed(min(tiltSps, MAX_SPEED_STEPS_S));
    const float limitDeg = (tiltDps > 0.0f) ? TILT_MAX_DEG : TILT_MIN_DEG;
    tiltStepper.moveTo(degToTiltSteps(limitDeg));
  }
}

static void handleCmd(const PtCmdFrame *cmd) {
  switch (cmd->op) {
    case PT_OP_MODE:
      if (cmd->mode == PT_MODE_IDLE) enterIdle();
      else if (cmd->mode == PT_MODE_HOMING) enterHoming();
      else if (cmd->mode == PT_MODE_RUNNING) enterRunning();
      return;

    case PT_OP_STATUS_REQ:
      txStatus(PT_OP_STATUS);
      return;

    case PT_OP_STOP:
      panStepper.stop();
      tiltStepper.stop();
      enterIdle();
      return;

    case PT_OP_VEL:
      if (currentState != ST_RUNNING) return;
      setVelocity(cmd->f0, cmd->f1);
      return;

    case PT_OP_ANG:
      if (currentState != ST_RUNNING) return;
      setAnglesDeg(cmd->f0, cmd->f1);
      return;

    case PT_OP_DXY:
      if (currentState != ST_RUNNING) return;
      moveDeltaDeg(cmd->f0, cmd->f1);
      return;

    case PT_OP_PAN:
      if (currentState != ST_RUNNING) return;
      setPanDeg(cmd->f0);
      return;

    case PT_OP_TILT:
      if (currentState != ST_RUNNING) return;
      setTiltDeg(cmd->f1);
      return;

    case PT_OP_HOME:
      if (currentState != ST_RUNNING) return;
      setAnglesDeg(0.0f, 0.0f);
      return;

    default:
      return;
  }
}

static void parseCmdFrame(const uint8_t *raw, size_t len) {
  if (len != PT_CMD_SIZE) return;

  PtCmdFrame cmd;
  memcpy(&cmd, raw, sizeof(cmd));
  if (cmd.magic != PT_MAGIC || cmd.ver != PT_VER) return;
  if (crc16_ccitt_false(raw, PT_CMD_CRC_LEN) != cmd.crc16) return;

  handleCmd(&cmd);
}

static void pollSerial() {
  while (Serial.available() > 0) {
    const uint8_t b = static_cast<uint8_t>(Serial.read());
    if (b == 0) {
      if (rxLen > 0) {
        uint8_t dec[64];
        const size_t n = cobs_decode(rxBuf, rxLen, dec, sizeof(dec));
        if (n > 0) parseCmdFrame(dec, n);
        rxLen = 0;
      }
      continue;
    }
    if (rxLen < sizeof(rxBuf)) {
      rxBuf[rxLen++] = b;
    } else {
      rxLen = 0;
    }
  }
}

static void tickRunningTimeout() {
  if (currentState != ST_RUNNING) return;
  if (lastAnyCmdMs == 0) return;
  if ((millis() - lastAnyCmdMs) >= CMD_IDLE_TIMEOUT_MS) {
    enterIdle();
  }
}

void setup() {
  Serial.begin(921600);
  delay(200);

  pinMode(PAN_EN_PIN, OUTPUT);
  pinMode(TILT_EN_PIN, OUTPUT);
  pinMode(LASER_PIN, OUTPUT);
  pinMode(TILT_ENDSTOP_PIN, INPUT_PULLUP);
  digitalWrite(PAN_EN_PIN, HIGH);
  digitalWrite(TILT_EN_PIN, HIGH);
  digitalWrite(LASER_PIN, LOW);

  panStepper.setMaxSpeed(MAX_SPEED_STEPS_S);
  panStepper.setAcceleration(ACCEL_STEPS_S2);
  tiltStepper.setMaxSpeed(MAX_SPEED_STEPS_S);
  tiltStepper.setAcceleration(ACCEL_STEPS_S2);

  panStepper.setEnablePin(PAN_EN_PIN);
  tiltStepper.setEnablePin(TILT_EN_PIN);

  currentState = ST_IDLE;
  txStatus(PT_OP_READY);
}

void loop() {
  pollSerial();

  switch (currentState) {
    case ST_IDLE:
      break;

    case ST_HOMING:
      tickHoming();
      panStepper.run();
      tiltStepper.run();
      break;

    case ST_RUNNING:
      tickRunningTimeout();
      panStepper.run();
      tiltStepper.run();
      enforcePanLimits();
      enforceTiltLimits();
      break;
  }

  updateLaser();
}
