/**
 * @file A4988.ino
 * @brief Ví dụ A4988: STEP/DIR/EN + MS1/MS2/MS3 điều khiển bằng thư viện.
 *
 * Arduino Mega2560:
 *   D22 -> STEP
 *   D23 -> DIR
 *   D42 -> ENABLE (A4988 nEN, active LOW)
 *   D46 -> MS1
 *   D47 -> MS2
 *   D48 -> MS3
 *   GND  -> GND logic A4988
 *
 * Lưu ý phần cứng:
 * - Cấp VMOT/motor supply đúng theo module và motor thực tế.
 * - Chỉnh giới hạn dòng của A4988 trước khi chạy motor.
 * - RESET/SLEEP phải ở trạng thái cho phép driver hoạt động theo mạch của bạn.
 */

#include <TungLam_Stepper.h>

constexpr uint8_t STEP_PIN = 22;
constexpr uint8_t DIR_PIN  = 23;
constexpr uint8_t EN_PIN   = 42;
constexpr uint8_t MS1_PIN  = 46;
constexpr uint8_t MS2_PIN  = 47;
constexpr uint8_t MS3_PIN  = 48;

TungLamStepper axis(STEP_PIN, DIR_PIN, EN_PIN);

void setup() {
  Serial.begin(115200);

  // A4988 thông dụng:
  // - ENABLE/nEN active LOW
  // - STEP pulse active HIGH
  // - DIR không đảo (đổi thành true nếu cơ khí quay ngược mong muốn)
  TungLamStepperDriverConfig driver(
      true,   // EN active LOW
      false,  // DIR không đảo
      4,      // STEP active width [us]
      4,      // DIR setup [us]
      true    // STEP active HIGH
  );

  TungLamStepperMotionConfig motion(
      5000,   // max speed [pulse/s]
      10000,  // acceleration [pulse/s^2]
      10000   // deceleration [pulse/s^2]
  );

  if (!axis.begin(driver, motion)) {
    Serial.print(F("A4988 begin failed: "));
    Serial.println(TungLamStepper::faultName(axis.fault()));
    return;
  }

  axis.setMotorFullStepsPerRevolution(200);  // motor 1.8°

  if (!axis.attachMicrostepPins(MS1_PIN, MS2_PIN, MS3_PIN)) {
    Serial.println(F("A4988 MS pins attach failed"));
    return;
  }

  // A4988: 1/16 microstep.
  if (!axis.setMicrostepMode(
          16,
          TungLamMicrostepDriverProfile::A4988)) {
    Serial.println(F("A4988 microstep config failed"));
    return;
  }

  Serial.print(F("A4988 pulses/rev = "));
  Serial.println(axis.pulsesPerOutputRevolution());

  axis.moveRevolutions(1.0f);
}

void loop() {
  // Không cần axis.run(); Timer1 phát STEP ở nền.

  if (axis.fault() != TungLamStepperFault::None) {
    Serial.print(F("Fault: "));
    Serial.println(TungLamStepper::faultName(axis.fault()));
  }
}
