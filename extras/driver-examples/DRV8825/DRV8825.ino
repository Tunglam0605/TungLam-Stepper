/**
 * @file DRV8825.ino
 * @brief Ví dụ DRV8825: STEP/DIR/nEN + MODE0/MODE1/MODE2.
 *
 * Arduino Mega2560:
 *   D22 -> STEP
 *   D23 -> DIR
 *   D42 -> nEN (active LOW)
 *   D46 -> MODE0
 *   D47 -> MODE1
 *   D48 -> MODE2
 *   GND  -> GND logic DRV8825
 *
 * Ví dụ chọn 1/32 microstep bằng profile DRV8825 tích hợp.
 */

#include <TungLam_Stepper.h>

constexpr uint8_t STEP_PIN = 22;
constexpr uint8_t DIR_PIN  = 23;
constexpr uint8_t EN_PIN   = 42;
constexpr uint8_t M0_PIN   = 46;
constexpr uint8_t M1_PIN   = 47;
constexpr uint8_t M2_PIN   = 48;

TungLamStepper axis(STEP_PIN, DIR_PIN, EN_PIN);

void setup() {
  Serial.begin(115200);

  TungLamStepperDriverConfig driver(
      true,   // nEN active LOW
      false,  // DIR không đảo
      4,      // STEP active width [us]
      4,      // DIR setup [us]
      true    // STEP active HIGH
  );

  TungLamStepperMotionConfig motion(
      6000,
      12000,
      12000
  );

  if (!axis.begin(driver, motion)) {
    Serial.print(F("DRV8825 begin failed: "));
    Serial.println(TungLamStepper::faultName(axis.fault()));
    return;
  }

  axis.setMotorFullStepsPerRevolution(200);

  if (!axis.attachMicrostepPins(M0_PIN, M1_PIN, M2_PIN)) {
    Serial.println(F("DRV8825 MODE pins attach failed"));
    return;
  }

  // DRV8825 hỗ trợ profile 1/32.
  if (!axis.setMicrostepMode(
          32,
          TungLamMicrostepDriverProfile::DRV8825)) {
    Serial.println(F("DRV8825 microstep config failed"));
    return;
  }

  axis.moveDegrees(180.0f);
}

void loop() {
  if (!axis.isRunning()) {
    // TODO: state machine của project.
  }
}
