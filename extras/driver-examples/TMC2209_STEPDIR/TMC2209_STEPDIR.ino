/**
 * @file TMC2209_STEPDIR.ino
 * @brief TMC2209 chạy ở STEP/DIR mode, không cấu hình UART.
 *
 * Arduino Mega2560:
 *   D22 -> STEP
 *   D23 -> DIR
 *   D42 -> EN (thường active LOW trên carrier phổ biến)
 *
 * Microstep phải được cấu hình bên ngoài thư viện:
 * - bằng MS pins / strap trên carrier; hoặc
 * - bằng UART trong phần code khác.
 *
 * TungLam_Stepper v0.1.x chỉ điều khiển motion STEP/DIR; không thay thế driver
 * UART configuration layer của TMC2209.
 */

#include <TungLam_Stepper.h>

TungLamStepper axis(22, 23, 42);

void setup() {
  Serial.begin(115200);

  TungLamStepperDriverConfig driver(
      true,   // EN active LOW trên carrier phổ biến
      false,
      4,
      4,
      true
  );

  TungLamStepperMotionConfig motion(
      8000,
      16000,
      16000
  );

  if (!axis.begin(driver, motion)) {
    Serial.print(F("TMC2209 STEP/DIR begin failed: "));
    Serial.println(TungLamStepper::faultName(axis.fault()));
    return;
  }

  axis.setMotorFullStepsPerRevolution(200);

  // Ví dụ driver/carrier/UART đã được cấu hình 1/16 ở nơi khác.
  axis.setMicrosteps(16);

  axis.moveToRevolutions(1.0f);
}

void loop() {
  // UART diagnostics/config có thể chạy ở đây nếu project cần,
  // miễn không chiếm Timer1 của TungLam_Stepper.
}
