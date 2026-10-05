/**
 * @file DM542.ino
 * @brief Ví dụ DM542 kiểu common-anode / sinking, STEP active LOW.
 *
 * Kiểu wiring minh họa:
 *   +5V         -> PUL+
 *   +5V         -> DIR+
 *   Arduino D22 -> PUL-
 *   Arduino D23 -> DIR-
 *   GND Arduino và nguồn logic phải có reference phù hợp với mạch giao tiếp.
 *
 * Với kiểu này MCU "kéo xuống" optocoupler:
 *   STEP idle HIGH
 *   pulse active LOW
 * nên stepActiveHigh=false.
 *
 * ENA trong ví dụ này không dùng để tránh giả định polarity của từng wiring.
 * Microstep đặt bằng DIP trên DM542.
 */

#include <TungLam_Stepper.h>

TungLamStepper axis(22, 23);

void setup() {
  Serial.begin(115200);

  TungLamStepperDriverConfig driver(
      true,   // EN không dùng
      false,  // không đảo DIR; đổi true nếu chiều cơ khí thực tế bị ngược
      6,      // STEP active width [us]
      6,      // DIR setup [us]
      false   // STEP active LOW / sinking
  );

  TungLamStepperMotionConfig motion(
      5000,
      10000,
      10000
  );

  if (!axis.begin(driver, motion)) {
    Serial.print(F("DM542 begin failed: "));
    Serial.println(TungLamStepper::faultName(axis.fault()));
    return;
  }

  axis.setMotorFullStepsPerRevolution(200);

  // Phải khớp đúng DIP trên driver.
  axis.setMicrosteps(16);

  // Ví dụ vitme 5 mm/vòng.
  axis.setTravelPerOutputRevolution(5.0f);

  axis.moveTravel(25.0f);  // +25 mm = 5 vòng output.
}

void loop() {
  // Có thể chạy sensor/Serial/state machine bình thường.
}
