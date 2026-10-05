/**
 * @file PositionTemplate.ino
 * @brief Template single-axis: copy project rồi thay cấu hình theo cơ cấu thật.
 *
 * Mega2560 Axis 1 baseline:
 * STEP=D22, DIR=D23, EN=D42.
 */

#include <TungLam_Stepper.h>

TungLamStepper axis(22, 23, 42);

bool reportedDone = false;

void setup() {
  Serial.begin(115200);

  if (!axis.begin()) {
    Serial.print(F("Stepper begin failed: "));
    Serial.println(TungLamStepper::faultName(axis.fault()));
    return;
  }

  // Motor 1.8° = 200 full-step/vòng.
  axis.setMotorFullStepsPerRevolution(200);

  // TB6600/DM542 đặt vi bước bằng DIP thì chỉ cần khai báo hệ số.
  axis.setMicrosteps(16);

  axis.setMaxSpeed(8000);       // pulse/s
  axis.setAcceleration(12000);  // pulse/s^2
  axis.setDeceleration(12000);  // pulse/s^2

  Serial.print(F("Timing ceiling [pulse/s]: "));
  Serial.println(axis.maximumStepRate());

  // Chạy tương đối +1 vòng đầu ra.
  axis.moveRevolutions(1.0f);
}

void loop() {
  // Không cần axis.run().
  // Timer1 tiếp tục phát STEP trong nền.

  if (!axis.isRunning() && !reportedDone) {
    axis.printState(Serial);
    reportedDone = true;

    // TODO: chuyển sang state tiếp theo của project tại đây.
  }
}
