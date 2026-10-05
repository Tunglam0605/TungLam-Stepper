/**
 * @file MultiAxisTemplate.ino
 * @brief 4 trục độc lập dùng chung Timer1 event scheduler.
 *
 * Recommended Mega2560 pin baseline:
 * AX1: STEP22 DIR23 EN42 MIN A8  MAX A9
 * AX2: STEP24 DIR25 EN43 MIN A10 MAX A11
 * AX3: STEP26 DIR27 EN44 MIN A12 MAX A13
 * AX4: STEP28 DIR29 EN45 MIN A14 MAX A15
 */

#include <TungLam_Stepper.h>

TungLamStepper axis1(22, 23, 42);
TungLamStepper axis2(24, 25, 43);
TungLamStepper axis3(26, 27, 44);
TungLamStepper axis4(28, 29, 45);

void configureAxis(TungLamStepper &axis) {
  axis.begin();
  axis.setMotorFullStepsPerRevolution(200);
  axis.setMicrosteps(16);
  axis.setMaxSpeed(6000);
  axis.setAcceleration(10000);
  axis.setDeceleration(10000);
}

void setup() {
  configureAxis(axis1);
  configureAxis(axis2);
  configureAxis(axis3);
  configureAxis(axis4);

  axis1.attachLimits(A8,  A9);
  axis2.attachLimits(A10, A11);
  axis3.attachLimits(A12, A13);
  axis4.attachLimits(A14, A15);

  // Các trục chạy độc lập, đồng thời, không block loop().
  axis1.moveTo( 3200);
  axis2.moveTo(-1600);
  axis3.moveTo( 6400);
  axis4.runContinuous(TungLamStepperDirection::Positive, 1500);
}

void loop() {
  // TODO: PS2, robot drive, sensor, CAN/UART/state machine vẫn chạy bình thường.

  if (!axis1.isRunning() && !axis2.isRunning() && !axis3.isRunning()) {
    // TODO: chuyển sang state tiếp theo.
  }
}
