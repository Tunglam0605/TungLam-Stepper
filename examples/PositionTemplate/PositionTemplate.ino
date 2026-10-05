/**
 * @file PositionTemplate.ino
 * @brief Template cơ bản: copy project rồi thay số bước/tốc độ theo cơ cấu thật.
 *
 * Baseline Mega2560 Axis 1:
 * STEP=D22, DIR=D23, EN=D42.
 */

#include <TungLam_Stepper.h>

TungLamStepper axis(22, 23, 42);

void setup() {
  axis.begin();

  // Motor 1.8°: 200 full-step/vòng.
  axis.setMotorFullStepsPerRevolution(200);

  // Driver đang đặt 1/16 microstep (bằng DIP/MS pin/UART tùy driver).
  axis.setMicrosteps(16);

  axis.setMaxSpeed(8000);      // pulse/s
  axis.setAcceleration(12000); // pulse/s^2
  axis.setDeceleration(12000); // pulse/s^2

  // 1 vòng đầu ra.
  axis.moveRevolutions(1.0f);
}

void loop() {
  // Không cần run() và không cần delay để tạo xung STEP.
  // Timer1 tiếp tục chạy motor ở nền.

  if (!axis.isRunning()) {
    // TODO: đặt state machine / PS2 / sensor / robot logic ở đây.
  }
}
