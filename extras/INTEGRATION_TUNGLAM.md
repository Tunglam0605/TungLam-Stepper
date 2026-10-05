# Tích hợp TungLam_PS2 + TungLam_OmniMecanum_4WD + TungLam_Stepper

## Resource ownership

```text
TungLam_PS2
  Hardware SPI D50..D53

TungLam_OmniMecanum_4WD
  Timer3 + Timer4
  PWM D5..D8
  DIR D30..D37

TungLam_Stepper
  Timer1
  recommended STEP/DIR D22..D29
```

Ba thư viện không phụ thuộc lẫn nhau ở core. Application mới quyết định arbitration.

## Pattern robot thực tế

Template đầy đủ:

```text
extras/integration-examples/ThreeLibraryRobot/ThreeLibraryRobot.ino
```

Flow:

```text
loop()
  ├─ ps2.update()
  ├─ robot.update()
  │
  ├─ mất PS2?
  │    ├─ robot.stop()
  │    └─ lift.stop()     <- idempotent fail-safe
  │
  ├─ right stick rotate priority
  ├─ left stick translation
  │
  └─ mechanism buttons
       ├─ Triangle -> stepper home
       ├─ R1       -> moveToTravel(+50 mm)
       ├─ R2       -> moveToTravel(0 mm)
       └─ Circle   -> emergencyStop
```

Stepper không cần run() trong loop.

## Cấu hình cơ khí trước khi dùng mm

```cpp
lift.setMotorFullStepsPerRevolution(200);
lift.setMicrosteps(16);
lift.setGearRatio(1.0f);
lift.setTravelPerOutputRevolution(8.0f);
```

Sau homing:

```cpp
lift.setSoftLimits(
    0,
    lift.travelToSteps(300.0f)
);

lift.moveToTravel(50.0f);
```

## Homing

Không nên bật soft-limit làm reference chính trước khi homing.

```cpp
TungLamStepperHomingConfig home(
    TungLamStepperDirection::Negative,
    1500,
    300,
    100,
    0,
    3,
    160000
);

lift.home(home);
```

maxPhaseSteps bảo vệ trường hợp switch đứt/hỏng.

## Fail-safe khi mất PS2

```cpp
if (!ps2.connected()) {
  robot.stop();

  if (lift.isRunning()) {
    lift.stop();
  }
  return;
}
```

stop() của Stepper là idempotent, nên application có thể đi qua nhánh fail-safe ở nhiều vòng loop mà không liên tục lập lại quãng dừng.

Nếu cơ cấu phải cắt motion tức thời vì nguy cơ cơ khí, dùng emergencyStop() thay stop().

## Quy tắc tránh xung đột

- Không gọi TungLam_Control_MotorV5::Init_Timer1() khi Stepper đang active.
- Không dùng analogWrite() D11/D12 sau khi Stepper claim Timer1.
- Giữ Servo trong Timer5 nếu có thể.
- Không dùng PS2 BitBang wiring RoboBall cũ D22/D24/D26/D28 nếu đồng thời muốn dùng Stepper baseline D22..D29. Hãy dùng Hardware SPI PS2 D50..D53.
