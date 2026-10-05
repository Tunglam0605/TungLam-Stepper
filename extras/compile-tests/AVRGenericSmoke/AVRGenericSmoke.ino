#include <TungLam_Stepper.h>

TungLamStepper axis(2, 4, 7);

void setup() {
  if (!axis.begin()) return;

  axis.setMotorFullStepsPerRevolution(200);
  axis.setMicrosteps(8);
  axis.setMaxSpeed(2000);
  axis.setAcceleration(4000);
  axis.setDeceleration(4000);

  axis.moveToDegrees(90.0f);
}

void loop() {
  if (!axis.isRunning()) {
    // Generic AVR smoke test only.
  }
}
