/**
 * @file TB6600.ino
 * @brief Ví dụ TB6600 kiểu DIP/manual microstep, STEP active HIGH.
 *
 * Kiểu wiring minh họa: common-cathode / drive HIGH
 *   Arduino D22 -> PUL+
 *   Arduino D23 -> DIR+
 *   GND         -> PUL-
 *   GND         -> DIR-
 *
 * ENA có thể bỏ qua nếu hệ thống của bạn luôn enable driver:
 *   TungLamStepper axis(D22, D23);  // không dùng EN pin
 *
 * Nếu module của bạn đấu kiểu common-anode/sinking:
 *   PUL+ / DIR+ -> +5V
 *   Arduino điều khiển PUL- / DIR-
 * thì đổi stepActiveHigh=false và kiểm tra DIR inversion theo wiring thực tế.
 *
 * Microstep và dòng motor được đặt bằng DIP trên TB6600; library chỉ cần biết
 * hệ số microstep để quy đổi rev/degree/mm chính xác.
 */

#include <TungLam_Stepper.h>

TungLamStepper axis(22, 23);  // STEP, DIR; không quản lý ENA.

void setup() {
  Serial.begin(115200);

  TungLamStepperDriverConfig driver(
      true,   // EN polarity không dùng vì không có EN pin
      false,  // DIR không đảo
      6,      // pulse width [us] - giá trị bảo thủ cho optocoupler
      6,      // DIR setup [us]
      true    // common-cathode: pulse active HIGH
  );

  TungLamStepperMotionConfig motion(
      4000,
      8000,
      8000
  );

  if (!axis.begin(driver, motion)) {
    Serial.print(F("TB6600 begin failed: "));
    Serial.println(TungLamStepper::faultName(axis.fault()));
    return;
  }

  axis.setMotorFullStepsPerRevolution(200);

  // Ví dụ DIP trên TB6600 đang đặt 1/16.
  // Library KHÔNG tự đổi DIP; chỉ khai báo đúng hệ số thực tế.
  axis.setMicrosteps(16);

  axis.moveRevolutions(2.0f);
}

void loop() {
  // Non-blocking.
}
