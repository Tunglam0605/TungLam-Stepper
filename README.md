# TungLam_Stepper

**TungLam_Stepper** là thư viện điều khiển động cơ bước chuẩn **STEP/DIR** cho Arduino AVR, được thiết kế cho robot và cơ cấu máy cần timing chính xác nhưng vẫn giữ `loop()` non-blocking.

Tác giả: **Nguyễn Khắc Tùng Lâm — Tùng Lâm Automation**

Thư viện được thiết kế để dùng độc lập hoặc kết hợp trực tiếp với:

- `TungLam_PS2`
- `TungLam_OmniMecanum_4WD`

Mục tiêu là tạo một stack Mega2560 rõ ownership tài nguyên:

```text
TungLam_PS2              -> Hardware SPI D50..D53
TungLam_Stepper          -> Timer1
TungLam_OmniMecanum_4WD  -> Timer3 + Timer4
```

---

## Điểm chính

- STEP pulse phát bằng **Timer1 Compare ISR**, không dùng vòng `delayMicroseconds()`.
- Không cần gọi `run()` liên tục để duy trì motor.
- GPIO hot-path dùng **cached AVR port register + bit mask**, không dùng `digitalWrite()`.
- Mặc định hỗ trợ tối đa **4 axis độc lập** trên cùng Timer1 scheduler.
- Position move:
  - `move()`
  - `moveTo()`
  - acceleration / cruise / deceleration.
- Continuous constant-speed:
  - `runContinuous()`
  - `stop()` giảm tốc.
- Fixed-point **Q24.8** cho recurrence interval để giảm lỗi lượng tử trên AVR.
- Hard limit MIN/MAX.
- Soft limit.
- Homing non-blocking hai lượt:
  - seek nhanh;
  - debounce;
  - backoff;
  - seek chậm;
  - set zero.
- Homing có **maxPhaseSteps** chống chạy vô hạn khi công tắc hỏng.
- Chống tràn position 32-bit.
- `stop()` idempotent và không tự kéo axis vượt target cũ.
- Timing driver cấu hình được:
  - pulse width;
  - STEP polarity;
  - DIR setup;
  - EN polarity;
  - DIR inversion.
- Mô hình cơ khí:
  - full-step/rev;
  - microsteps;
  - gear ratio;
  - travel/rev.
- Hỗ trợ relative và absolute theo:
  - pulse;
  - vòng;
  - độ;
  - mm hoặc đơn vị tuyến tính tùy chọn.
- Microstep:
  - Manual/DIP cho TB6600, DM542 và driver STEP/DIR công nghiệp;
  - profile GPIO tích hợp cho A4988;
  - profile GPIO tích hợp cho DRV8825.
- `printState()` và tên fault/mode dạng PROGMEM để debug ít tốn SRAM.
- `end()` trả Timer1 về trạng thái trước khi library lấy timer, khi axis cuối được gỡ.

---

## Vì sao không dùng cách RoboBall cũ?

Code RoboBall cũ đã dùng thanh ghi trực tiếp để GPIO nhanh:

```cpp
PORTE |=  (1 << PE4);
PORTE &= ~(1 << PE4);
```

nhưng STEP vẫn được phát bằng:

```cpp
for (...) {
  STEP_HIGH;
  delayMicroseconds(delayTime);

  STEP_LOW;
  delayMicroseconds(delayTime);
}
```

Trong thời gian đó CPU bị giữ trong vòng lặp.

`TungLam_Stepper` thay phần này bằng:

```text
Timer1 Compare A -> STEP rising event
Timer1 Compare B -> STEP falling event
```

Application vẫn có thể tiếp tục:

```cpp
ps2.update();
robot.update();
// sensor / Serial / state machine...
```

trong khi stepper đang chạy.

---

# Mega2560 pin baseline

Đây là **pinout khuyến nghị**, không phải chân bắt buộc.

| Axis | STEP | DIR | EN | MIN/HOME | MAX |
|---|---:|---:|---:|---:|---:|
| Axis 1 | D22 / PA0 | D23 / PA1 | D42 / PL7 | A8 / PK0 | A9 / PK1 |
| Axis 2 | D24 / PA2 | D25 / PA3 | D43 / PL6 | A10 / PK2 | A11 / PK3 |
| Axis 3 | D26 / PA4 | D27 / PA5 | D44 / PL5 | A12 / PK4 | A13 / PK5 |
| Axis 4 | D28 / PA6 | D29 / PA7 | D45 / PL4 | A14 / PK6 | A15 / PK7 |

STEP/DIR được xếp liền trên PORTA và limit trên PORTK để thuận tiện cho shield/layout và tối ưu thanh ghi.

Chi tiết: [extras/PINOUT_MEGA2560.md](extras/PINOUT_MEGA2560.md)

---

# Quick start

```cpp
#include <TungLam_Stepper.h>

TungLamStepper lift(22, 23, 42);

void setup() {
  Serial.begin(115200);

  if (!lift.begin()) {
    Serial.println(TungLamStepper::faultName(lift.fault()));
    return;
  }

  lift.setMotorFullStepsPerRevolution(200);
  lift.setMicrosteps(16);

  lift.setMaxSpeed(8000);       // pulse/s
  lift.setAcceleration(12000);  // pulse/s^2
  lift.setDeceleration(12000);  // pulse/s^2

  lift.moveRevolutions(1.0f);
}

void loop() {
  // Không cần lift.run().
}
```

---

# STEP/DIR driver configuration

```cpp
TungLamStepperDriverConfig driver(
    true,   // EN active LOW
    false,  // DIR không đảo
    4,      // STEP active width [us]
    4,      // DIR setup [us]
    true    // STEP active HIGH; false = pulse LOW / sinking
);

TungLamStepperMotionConfig motion(
    10000,  // max speed [pulse/s]
    20000,  // acceleration [pulse/s^2]
    20000   // deceleration [pulse/s^2]
);

axis.begin(driver, motion);
```

Library precompute timing sang timer tick khi cấu hình. Đường ISR không thực hiện phép chia 64-bit để đổi microsecond.

Với driver kiểu **common-anode / sinking input**, có thể đặt `stepActiveHigh=false`: chân STEP idle HIGH và pulse được tạo bằng mức LOW. Tham số này nằm cuối constructor nên code 4 tham số cũ vẫn giữ nguyên hành vi active-HIGH.

---

# Driver tương thích

| Driver | STEP/DIR | EN | Microstep |
|---|---|---|---|
| A4988 | ✅ | ✅ | GPIO MS1/MS2/MS3 hoặc manual |
| DRV8825 | ✅ | ✅ | GPIO MODE0/MODE1/MODE2 hoặc manual |
| TB6600 | ✅ | ✅ | DIP/manual |
| DM542 | ✅ | ✅ | DIP/manual |
| TMC2208/2209 ở STEP/DIR mode | ✅ | tùy wiring | manual |
| Driver STEP/DIR công nghiệp khác | ✅ | tùy driver | manual |

TMC UART/SPI configuration adapter chưa nằm trong v0.1; STEP/DIR mode vẫn dùng được.

---

# Microstep

## TB6600 / DM542 / DIP switch

Driver tự đặt vi bước bằng DIP, library chỉ cần biết hệ số để quy đổi đơn vị:

```cpp
axis.setMicrosteps(16);
```

## A4988

```cpp
axis.attachMicrostepPins(MS1_PIN, MS2_PIN, MS3_PIN);
axis.setMicrostepMode(
    16,
    TungLamMicrostepDriverProfile::A4988
);
```

## DRV8825

```cpp
axis.attachMicrostepPins(MODE0_PIN, MODE1_PIN, MODE2_PIN);
axis.setMicrostepMode(
    32,
    TungLamMicrostepDriverProfile::DRV8825
);
```

Có thể bỏ profile tích hợp và ghi mức logic tùy ý:

```cpp
axis.setMicrostepPinLevels(true, false, true);
axis.setMicrosteps(32);
```

---

# Mô hình cơ khí

Ví dụ:

- motor 1.8° = 200 full-step/rev;
- microstep = 1/16;
- gear ratio = 1:1;
- vitme lead = 8 mm/rev.

```cpp
axis.setMotorFullStepsPerRevolution(200);
axis.setMicrosteps(16);
axis.setGearRatio(1.0f);
axis.setTravelPerOutputRevolution(8.0f);
```

Khi đó:

```text
200 × 16 = 3200 pulse/rev
3200 / 8 = 400 pulse/mm
```

---

# Relative motion

```cpp
axis.move(3200);           // +3200 pulse
axis.moveRevolutions(1);   // +1 vòng output
axis.moveDegrees(90);      // +90°
axis.moveTravel(50);       // +50 mm nếu travelUnit đang là mm
```

# Absolute motion

```cpp
axis.moveTo(6400);
axis.moveToRevolutions(2);
axis.moveToDegrees(180);
axis.moveToTravel(150);
```

Các hàm chuyển đổi cũng có sẵn:

```cpp
int32_t p = axis.travelToSteps(50.0f);
float mm  = axis.stepsToTravel(p);
```

---

# Homing

```cpp
axis.attachLimits(A8, A9, true, true);

TungLamStepperHomingConfig home(
    TungLamStepperDirection::Negative,
    2000,    // fast seek [pulse/s]
    400,     // slow re-approach [pulse/s]
    100,     // backoff [pulse]
    0,       // home position
    3,       // số mẫu liên tiếp xác nhận switch
    160000   // max pulse cho MỖI pha; 0 = không giới hạn
);

axis.home(home);
```

State machine:

```text
SeekFast
   ↓
switch active đủ N mẫu
   ↓
Backoff
   ↓
switch release đủ N mẫu
   ↓
backoff thêm X pulse
   ↓
SeekSlow
   ↓
switch active đủ N mẫu
   ↓
position = homePosition
   ↓
Homed
```

Trong lúc debounce active, library **không tiếp tục đẩy STEP sâu hơn vào công tắc**.

Nếu một pha vượt `maxPhaseSteps`, axis dừng với:

```text
HomingTravelExceeded
```

Soft-limit được bỏ qua trong homing vì vị trí tuyệt đối chưa đáng tin cậy trước khi tìm home.

---

# Hard limit và soft limit

```cpp
axis.attachLimits(A8, A9);

axis.setCurrentPosition(0);
axis.setSoftLimits(0, axis.travelToSteps(300.0f));
```

Hard limit chỉ chặn hướng đang đi **vào** switch. Hướng đi ra khỏi switch vẫn được phép để recovery.

Soft limit cũng cho phép recovery nếu position hiện tại đang ở ngoài envelope: hướng đi **về vùng hợp lệ** được phép, hướng đi xa hơn ra ngoài bị chặn. Soft limit được kiểm tra trước lệnh và trước mỗi STEP event.

---

# Stop semantics

## Giảm tốc

```cpp
axis.stop();
```

- position/continuous motion: giảm tốc rồi dừng;
- không tự kéo axis vượt target cũ;
- tôn trọng soft-limit;
- gọi `stop()` lặp lại nhiều lần là an toàn vì stop là **idempotent**;
- homing: `stop()` chuyển thành dừng tức thời để state homing không bị biến dạng.

## Dừng tức thời

```cpp
axis.emergencyStop();
```

Ngắt lịch phát STEP ngay, không áp profile giảm tốc.

---

# Continuous motion

```cpp
axis.runContinuous(
    TungLamStepperDirection::Positive,
    1500
);
```

`runContinuous()` là constant-speed mode và không cho tốc độ vượt:

- `motionConfig.maxSpeedStepsPerSecond`;
- trần timing tính từ pulse-width + Timer1 guard.

Đọc trần timing:

```cpp
uint32_t ceiling = axis.maximumStepRate();
```

> `maximumStepRate()` là **trần timing lý thuyết**. Tốc độ ổn định thực tế phụ thuộc số axis cùng chạy, ISR load, driver và phần cứng.

---

# Multi-axis

```cpp
TungLamStepper x(22, 23, 42);
TungLamStepper y(24, 25, 43);
TungLamStepper z(26, 27, 44);

x.begin();
y.begin();
z.begin();

x.moveTo(10000);
y.moveTo(-5000);
z.runContinuous(TungLamStepperDirection::Positive, 1000);
```

Các axis dùng chung event scheduler nhưng chạy độc lập.

**Synchronized-arrival group planner** chưa nằm trong v0.1 và được giữ cho roadmap sau để core single-axis không bị phình.

---

# Debug

```cpp
axis.printState(Serial);
```

Ví dụ:

```text
mode=Position running=1 stopping=0 enabled=1
pos=1280 target=20000 interval_ticks=456
fault=None homed=1 home_state=Complete
```

Tên fault không cần tự viết switch:

```cpp
Serial.println(
    TungLamStepper::faultName(axis.fault())
);
```

---

# Timer1 ownership

`begin()` chỉ lấy Timer1 khi timer đang:

1. ở cấu hình mặc định của Arduino AVR core; hoặc
2. chưa được cấu hình.

Nếu Timer1 đã bị thư viện khác thay đổi mode/output/interrupt, `begin()` fail với:

```text
Timer1Conflict
```

Khi gọi:

```cpp
axis.end();
```

và đây là axis Stepper cuối cùng, library khôi phục register Timer1 đã lưu trước lúc claim.

Khi đang dùng Stepper:

- không gọi legacy `TungLam_Control_MotorV5::Init_Timer1()`;
- không dùng PWM Timer1 D11/D12;
- tránh để Servo library mở rộng tới Timer1.

---

# Kết hợp ba thư viện TungLam

Ví dụ tích hợp thật được giữ ngoài Arduino IDE menu:

```text
extras/integration-examples/ThreeLibraryRobot/
```

Nó kiểm tra cùng lúc:

```text
PS2 SPI
   ↓
Mecanum Drive -> Timer3/Timer4
   +
Stepper Lift  -> Timer1
```

Joystick phải vẫn giữ priority quay như baseline RoboBall.

---

# Examples chính

Menu Arduino IDE chỉ giữ 3 template:

1. **PositionTemplate** — single-axis position/revolution.
2. **LinearAxisTemplate** — homing + vitme + soft-limit + mm.
3. **MultiAxisTemplate** — nhiều axis dùng chung Timer1.

Triết lý:

> **example = skeleton project dùng thật, không phải mỗi API một demo nhỏ.**

## Ví dụ theo từng driver

Để không làm menu Arduino IDE bị rác, các ví dụ phần cứng theo driver nằm trong:

```text
extras/driver-examples/
```

Có sẵn:

- **A4988** — STEP/DIR/nEN + MS1/MS2/MS3, profile microstep tích hợp.
- **DRV8825** — STEP/DIR/nEN + MODE0/1/2, ví dụ 1/32.
- **TB6600** — DIP/manual microstep, ví dụ STEP active-HIGH.
- **DM542** — DIP/manual microstep, ví dụ common-anode/sinking active-LOW.
- **TMC2209 STEP/DIR** — motion STEP/DIR, UART configuration để subsystem khác quản lý.

Xem [Driver-specific examples](extras/driver-examples/README.md) để chọn đúng wiring/polarity.

---

# Chú thích API trong Arduino IDE

Header public dùng Doxygen tiếng Việt theo cùng chuẩn với hai thư viện TungLam trước:

- mọi public API có `@brief`;
- API có tham số có `@param`;
- API trả dữ liệu có `@return`;
- đơn vị như `pulse/s`, `pulse/s²`, `us`, `độ`, `rev`, travel unit được ghi rõ;
- enum fault/mode/homing và resource Timer1 được giải thích ngay trong source.

Gate `tools/check_api_docs.py` kiểm tự động toàn bộ public API để tránh thêm hàm mới mà quên comment.

# Validation

Local software gates hiện được thiết kế gồm:

- architecture audit;
- motion-math regression;
- Arduino Mega2560 compile;
- `--warnings all`;
- three-library integration compile;
- GitHub Actions Arduino Lint strict.

Hardware validation vẫn cần trước khi gọi v1.0 hardware-stable:

- STEP/DIR oscilloscope hoặc logic analyzer;
- A4988 / DRV8825;
- TB6600 hoặc DM542;
- limit/homing thật;
- nhiều axis cùng chạy;
- PS2 + Mecanum + Stepper cùng lúc.

---

# Tài liệu kỹ thuật

- [Architecture](extras/ARCHITECTURE.md)
- [Mega2560 Pinout](extras/PINOUT_MEGA2560.md)
- [TungLam ecosystem integration](extras/INTEGRATION_TUNGLAM.md)
- [Technical references](extras/REFERENCES.md)
- [Validation matrix](extras/VALIDATION.md)

---

# License

MIT License.
