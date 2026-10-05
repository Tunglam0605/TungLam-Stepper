/**
 * @file LinearAxisTemplate.ino
 * @brief Template vitme/belt: home -> zero -> soft-limit -> chạy theo mm.
 *
 * Mega2560 Axis 1 baseline:
 * STEP=D22, DIR=D23, EN=D42
 * MIN/HOME=A8, MAX=A9
 */

#include <TungLam_Stepper.h>

TungLamStepper slide(22, 23, 42);

bool moveAfterHomeIssued = false;
bool initOk = false;

void setup() {
  Serial.begin(115200);

  if (!slide.begin()) {
    Serial.print(F("Stepper begin failed: "));
    Serial.println(TungLamStepper::faultName(slide.fault()));
    return;
  }

  slide.setMotorFullStepsPerRevolution(200);

  // CÁCH 1 - TB6600/DM542 hoặc driver đặt vi bước bằng DIP:
  slide.setMicrosteps(16);

  // CÁCH 2 - A4988/DRV8825 điều khiển MS pins bằng phần mềm:
  // slide.attachMicrostepPins(46, 47, 48);
  // slide.setMicrostepMode(16, TungLamMicrostepDriverProfile::A4988);

  slide.setGearRatio(1.0f);

  // Ví dụ vitme lead = 8 mm/vòng output.
  slide.setTravelPerOutputRevolution(8.0f);

  slide.setMaxSpeed(12000);
  slide.setAcceleration(18000);
  slide.setDeceleration(18000);

  // Limit active LOW + INPUT_PULLUP.
  if (!slide.attachLimits(A8, A9, true, true)) {
    Serial.print(F("Limit setup failed: "));
    Serial.println(TungLamStepper::faultName(slide.fault()));
    return;
  }

  TungLamStepperHomingConfig homeConfig(
      TungLamStepperDirection::Negative,
      2000,    // tìm nhanh [pulse/s]
      400,     // bắt lại chậm [pulse/s]
      100,     // lùi khỏi switch thêm 100 pulse
      0,       // home = position 0
      3,       // xác nhận 3 mẫu liên tiếp
      160000   // giới hạn pulse cho MỖI pha homing
  );

  if (!slide.home(homeConfig)) {
    Serial.print(F("Homing start failed: "));
    Serial.println(TungLamStepper::faultName(slide.fault()));
    return;
  }

  initOk = true;
}

void loop() {
  if (!initOk) return;

  // Không cần slide.run(). Timer1 tự phát STEP ở nền.

  if (slide.fault() != TungLamStepperFault::None) {
    Serial.print(F("Stepper fault: "));
    Serial.println(TungLamStepper::faultName(slide.fault()));
    initOk = false;
    return;
  }

  if (slide.isHomed() && !slide.isRunning() && !moveAfterHomeIssued) {
    // Sau khi home xong mới tin position tuyệt đối và bật soft-limit.
    slide.setSoftLimits(
        0,
        slide.travelToSteps(300.0f));  // hành trình 0..300 mm

    slide.moveToTravel(50.0f);
    moveAfterHomeIssued = true;
  }
}
