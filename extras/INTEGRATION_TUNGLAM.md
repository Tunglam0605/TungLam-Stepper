# Tích hợp ba thư viện TungLam trên Mega2560

## Kiến trúc

```text
TungLam_PS2
    │
    ▼
Application state / arbitration
    ├─────────> TungLam_OmniMecanum_4WD
    │                 └─ Timer3 + Timer4
    │
    └─────────> TungLam_Stepper
                      └─ Timer1
```

PS2 dùng Hardware SPI D50..D53 và không sở hữu timer của hai motion library.

## Pattern khuyến nghị

```cpp
#include <TungLam_PS2.h>
#include <TungLam_OmniMecanum_4WD.h>
#include <TungLam_Stepper.h>

TungLamPS2 ps2;
TungLamDrive4WD robot;
TungLamStepper lift(22, 23, 42);

void setup() {
  robot.begin(TungLamPwmMode::High7k8Hz);
  ps2.begin(53);

  lift.begin();
  lift.attachLimits(A8, A9);
  lift.setMicrosteps(16);
}

void loop() {
  ps2.update();
  robot.update();

  // Drive logic.
  // Stepper không cần run(): Timer1 vẫn phát xung ở nền.

  if (ps2.pressed(PS2Button::R1) && !lift.isRunning()) {
    lift.moveTravel(50.0f);
  }

  if (!ps2.connected()) {
    robot.stop();
    lift.stop();
  }
}
```

## Quy tắc ownership

Không gọi `TungLam_Control_MotorV5::Init_Timer1()` khi dùng `TungLam_Stepper`.

Nếu dùng Servo trên Mega, giữ số lượng servo trong nhóm Timer5 trước. Không để Servo mở rộng sang Timer1.

Các integration sketch không đặt trong `examples/` để `TungLam_Stepper` không biến PS2/Drive thành dependency bắt buộc.
