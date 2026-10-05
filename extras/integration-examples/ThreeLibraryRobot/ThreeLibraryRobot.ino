/**
 * @file ThreeLibraryRobot.ino
 * @brief Integration regression/template: PS2 + Mecanum/Omni + Stepper.
 *
 * Resource ownership on Arduino Mega2560:
 * - TungLam_PS2                : SPI D50..D53
 * - TungLam_OmniMecanum_4WD   : Timer3 + Timer4, D5..D8, D30..D37
 * - TungLam_Stepper           : Timer1, STEP/DIR D22/D23, EN D42
 * - Stepper limits            : A8/A9
 *
 * Control policy:
 * - JOY phải quay và có priority cao hơn JOY trái.
 * - JOY trái tiến/lùi/ngang.
 * - TRIANGLE: home cơ cấu stepper.
 * - R1: đi tới +50 mm.
 * - R2: đi tới 0 mm.
 */

#include <TungLam_PS2.h>
#include <TungLam_OmniMecanum_4WD.h>
#include <TungLam_Stepper.h>

TungLamPS2 ps2;
TungLamDrive4WD robot;
TungLamStepper lift(22, 23, 42);

constexpr uint8_t PS2_CS_PIN = 53;
constexpr uint8_t MOVE_SPEED = 180;
constexpr uint8_t TURN_SPEED = 155;

bool stepperReady = false;

void handleDrive();
void handleMechanism();
void safeStop();

void setup() {
  Serial.begin(115200);

  robot.begin(TungLamPwmMode::High7k8Hz);
  robot.setChassis(TungLamChassis::MecanumX);
  robot.stop();

  ps2.begin(PS2_CS_PIN);
  ps2.setPollRate(PS2PollRate::Hz50);

  stepperReady = lift.begin();
  if (!stepperReady) {
    Serial.print(F("Stepper init failed: "));
    Serial.println(TungLamStepper::faultName(lift.fault()));
    return;
  }

  lift.setMotorFullStepsPerRevolution(200);
  lift.setMicrosteps(16);
  lift.setGearRatio(1.0f);
  lift.setTravelPerOutputRevolution(8.0f);  // Ví dụ vitme lead 8 mm/rev.

  lift.setMaxSpeed(6000);
  lift.setAcceleration(10000);
  lift.setDeceleration(10000);

  lift.attachLimits(A8, A9, true, true);
}

void loop() {
  ps2.update();
  robot.update();

  if (!ps2.connected()) {
    safeStop();
    return;
  }

  handleDrive();

  if (stepperReady) {
    handleMechanism();
  }
}

void handleDrive() {
  // Priority 1: joystick phải điều khiển quay.
  switch (ps2.rightDirection()) {
    case PS2StickDirection::Left:
      robot.rotateLeft(TURN_SPEED);
      return;

    case PS2StickDirection::Right:
      robot.rotateRight(TURN_SPEED);
      return;

    default:
      break;
  }

  // Priority 2: joystick trái điều khiển tịnh tiến.
  switch (ps2.leftDirection()) {
    case PS2StickDirection::Up:
      robot.forward(MOVE_SPEED);
      break;

    case PS2StickDirection::Down:
      robot.backward(MOVE_SPEED);
      break;

    case PS2StickDirection::Left:
      robot.strafeLeft(MOVE_SPEED);
      break;

    case PS2StickDirection::Right:
      robot.strafeRight(MOVE_SPEED);
      break;

    default:
      robot.stop();
      break;
  }
}

void handleMechanism() {
  if (ps2.pressed(PS2Button::Triangle) && !lift.isRunning()) {
    TungLamStepperHomingConfig home(
        TungLamStepperDirection::Negative,
        1500,    // seek nhanh [pulse/s]
        300,     // seek chậm [pulse/s]
        100,     // backoff [pulse]
        0,       // zero
        3,       // debounce samples
        160000   // max pulse cho mỗi pha homing
    );
    lift.home(home);
  }

  if (ps2.pressed(PS2Button::R1) && lift.isHomed() && !lift.isRunning()) {
    const int32_t maxTravelSteps = lift.travelToSteps(300.0f);
    lift.setSoftLimits(0, maxTravelSteps);
    lift.moveToTravel(50.0f);
  }

  if (ps2.pressed(PS2Button::R2) && lift.isHomed() && !lift.isRunning()) {
    lift.moveToTravel(0.0f);
  }

  if (ps2.pressed(PS2Button::Circle)) {
    lift.emergencyStop();
  }
}

void safeStop() {
  robot.stop();

  // Không dùng emergencyStop() mỗi loop vì mất PS2 có thể chỉ là nhiễu ngắn.
  // stop() sẽ giảm tốc nếu axis đang chạy position/continuous.
  if (stepperReady && lift.isRunning()) {
    lift.stop();
  }
}
