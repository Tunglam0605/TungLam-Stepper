# TungLam_Stepper

Thư viện điều khiển stepper STEP/DIR cho hệ robot Arduino AVR của **Nguyễn Khắc Tùng Lâm / Tùng Lâm Automation**.

## Mục tiêu

`TungLam_Stepper` được thiết kế để dùng cùng:

- `TungLam_PS2`
- `TungLam_OmniMecanum_4WD`

và vẫn đủ độc lập để dùng cho bàn trượt, vitme, nâng/hạ, mâm xoay, cơ cấu gắp, bắn bóng hoặc machine axis.

### Khác code RoboBall cũ

RoboBall cũ đã tối ưu I/O bằng `PORTE/PORTL`, nhưng pulse STEP vẫn là:

```cpp
for (...) {
  STEP_HIGH;
  delayMicroseconds(...);
  STEP_LOW;
  delayMicroseconds(...);
}
```

Thư viện mới chuyển thành **Timer1 event engine**, nên không block PS2/robot/sensor trong lúc stepper chạy.

## Tính năng nền v0.1.0

- Timer1 non-blocking STEP pulse engine.
- Không cần `run()` để duy trì motor.
- Cached AVR port-register I/O, không dùng `digitalWrite()` trên hot path.
- Tối đa 4 axis độc lập mặc định.
- Position: `move()`, `moveTo()`.
- Unit helpers: revolution, degree, travel/mm.
- Trapezoidal acceleration/deceleration.
- Continuous constant-speed mode.
- Controlled stop cho position move.
- Emergency stop.
- MIN/MAX hard limits.
- Software limits.
- STEP/DIR/ENA polarity configurable.
- Full-step/rev + microsteps + gear ratio + travel/rev.
- Pin tùy chọn; pin baseline Mega chỉ là fast/reference layout.

## Mega2560 baseline

| Axis | STEP | DIR | EN | MIN | MAX |
|---|---:|---:|---:|---:|---:|
| 1 | D22 | D23 | D42 | A8 | A9 |
| 2 | D24 | D25 | D43 | A10 | A11 |
| 3 | D26 | D27 | D44 | A12 | A13 |
| 4 | D28 | D29 | D45 | A14 | A15 |

Chi tiết resource map: [extras/PINOUT_MEGA2560.md](extras/PINOUT_MEGA2560.md).

## Quick start

```cpp
#include <TungLam_Stepper.h>

TungLamStepper lift(22, 23, 42);

void setup() {
  lift.begin();

  lift.setMotorFullStepsPerRevolution(200);
  lift.setMicrosteps(16);

  lift.setMaxSpeed(8000);
  lift.setAcceleration(12000);
  lift.setDeceleration(12000);

  lift.moveTo(6400);
}

void loop() {
  // PS2 / robot drive / sensor logic tiếp tục chạy.
}
```

## Driver linh hoạt

Core không khóa vào A4988/TB6600/DM542.

```cpp
TungLamStepperDriverConfig driver(
    true,   // EN active LOW
    false,  // DIR không đảo
    4,      // STEP pulse width [us]
    4       // DIR setup [us]
);

TungLamStepperMotionConfig motion(
    10000,  // max pulse/s
    20000,  // accel pulse/s^2
    20000   // decel pulse/s^2
);

axis.begin(driver, motion);
```

Với TB6600/DM542, DIP microstep được khai báo cho motion model:

```cpp
axis.setMicrosteps(16);
```

Với A4988/DRV8825, có thể tùy chọn cho library điều khiển ba chân microstep:

```cpp
axis.attachMicrostepPins(MS1_PIN, MS2_PIN, MS3_PIN);
axis.setMicrostepMode(16, TungLamMicrostepDriverProfile::A4988);
```

Core motion vẫn không phụ thuộc driver; microstep GPIO chỉ là adapter tùy chọn.

## Linear mechanism

```cpp
axis.setMotorFullStepsPerRevolution(200);
axis.setMicrosteps(16);
axis.setGearRatio(1.0f);
axis.setTravelPerOutputRevolution(8.0f); // vitme 8 mm/rev

axis.moveTravel(50.0f); // +50 mm
```

## Limits và homing

```cpp
axis.attachLimits(A8, A9); // active LOW + pull-up mặc định

TungLamStepperHomingConfig home(
    TungLamStepperDirection::Negative,
    2000,  // seek nhanh
    400,   // re-approach chậm
    100,   // backoff pulse
    0,     // zero position
    3      // debounce samples
);

axis.home(home);
```

Homing chạy hoàn toàn non-blocking:

```text
seek nhanh
   ↓
limit xác nhận đủ N mẫu
   ↓
đảo chiều thoát switch
   ↓
backoff thêm một khoảng
   ↓
tiếp cận lại tốc độ chậm
   ↓
limit xác nhận đủ N mẫu
   ↓
set zero
```

Sau khi `isHomed()==true`, application mới nên bật soft limit. Hard limit bình thường chỉ chặn hướng đang đi **vào** công tắc; hướng thoát khỏi limit vẫn được phép.

## Resource contract

Khi `TungLam_Stepper::begin()` lấy Timer1:

- Timer3/Timer4 vẫn dành cho `TungLam_OmniMecanum_4WD`;
- SPI D50..D53 vẫn dành cho `TungLam_PS2`;
- không dùng legacy `Init_Timer1()`;
- không dùng `analogWrite()` D11/D12 trong cùng project;
- thư viện khác đang sở hữu Timer1 interrupt sẽ làm `begin()` thất bại thay vì âm thầm phá timer.

## Examples

Chỉ giữ ba template chính:

- `PositionTemplate`
- `LinearAxisTemplate`
- `MultiAxisTemplate`

## Roadmap

- v0.1: timer engine + position/continuous + limits + two-pass homing + microstep adapters + mechanism model.
- v0.2: synchronized-arrival multi-axis group planner.
- v0.3: queued motion / richer mechanism presets.
- v0.4: optional TMC UART/SPI driver adapters.
- v0.5: full PS2 + Drive + Stepper hardware bench regression.
- v1.0: sau hardware validation thật.

## Tài liệu kỹ thuật

Các datasheet và application note dùng để kiểm chứng timing/microstep được liệt kê tại `extras/REFERENCES.md`.

## Trạng thái

Đây là **development baseline**, chưa phát hành Arduino Library Manager và chưa được coi là hardware-stable.
