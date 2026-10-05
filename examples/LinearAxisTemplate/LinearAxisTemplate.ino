/**
 * @file LinearAxisTemplate.ino
 * @brief Template vitme/belt/linear axis: home -> zero -> soft-limit -> chạy theo mm.
 *
 * Axis 1 baseline:
 * STEP D22, DIR D23, EN D42
 * MIN/HOME A8, MAX A9
 */

#include <TungLam_Stepper.h>

TungLamStepper slide(22, 23, 42);

bool moveAfterHomeIssued = false;

void setup() {
  slide.begin();

  slide.setMotorFullStepsPerRevolution(200);

  // CÁCH 1 - TB6600/DM542 hoặc driver đặt vi bước bằng DIP:
  slide.setMicrosteps(16);

  // CÁCH 2 - A4988/DRV8825 nếu muốn điều khiển MS pins bằng phần mềm:
  // slide.attachMicrostepPins(46, 47, 48);
  // slide.setMicrostepMode(16, TungLamMicrostepDriverProfile::A4988);

  slide.setGearRatio(1.0f);

  // Ví dụ vitme lead = 8 mm/vòng.
  slide.setTravelPerOutputRevolution(8.0f);

  slide.setMaxSpeed(12000);
  slide.setAcceleration(18000);
  slide.setDeceleration(18000);

  // Limit active LOW + INPUT_PULLUP.
  slide.attachLimits(A8, A9, true, true);

  TungLamStepperHomingConfig homeConfig(
      TungLamStepperDirection::Negative,
      2000, // tìm nhanh [pulse/s]
      400,  // bắt lại chậm [pulse/s]
      100,  // lùi khỏi switch thêm 100 pulse
      0,    // home = position 0
      3     // xác nhận 3 mẫu liên tiếp
  );

  slide.home(homeConfig);
}

void loop() {
  // Không cần slide.run(). Timer1 tự phát STEP ở nền.

  if (slide.isHomed() && !slide.isRunning() && !moveAfterHomeIssued) {
    const float pulsesPerMm =
        slide.pulsesPerOutputRevolution() /
        slide.travelPerOutputRevolution();

    // Sau khi home xong mới bật giới hạn mềm 0..300 mm.
    slide.setSoftLimits(
        0,
        static_cast<int32_t>(300.0f * pulsesPerMm));

    slide.moveTravel(50.0f);
    moveAfterHomeIssued = true;
  }

  if (slide.fault() != TungLamStepperFault::None) {
    // TODO: báo lỗi / HMI / buzzer / state machine.
  }
}
