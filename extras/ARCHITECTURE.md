# TungLam_Stepper Architecture

## 1. Mục tiêu kiến trúc

TungLam_Stepper tách rõ bốn lớp:

```text
Application
  ├─ PS2 / button / Serial / ROS2 bridge / state machine
  │
  └─ TungLamStepper public API
       │
       ├─ Mechanism model
       │    full-step/rev
       │    microstep
       │    gear ratio
       │    travel/rev
       │
       ├─ Motion planner
       │    position target
       │    trapezoidal / triangular profile
       │    Q24.8 interval recurrence
       │    controlled stop
       │
       ├─ Safety / reference
       │    hard MIN/MAX
       │    soft limits
       │    position overflow
       │    two-pass homing
       │    homing phase travel guard
       │
       └─ AVR timing HAL
            ├─ cached GPIO register/mask
            └─ Timer1 event scheduler
                 ├─ COMPA -> STEP rising event
                 └─ COMPB -> STEP falling event
```

Core motion không phụ thuộc PS2, chassis hay một loại driver cụ thể.

## 2. Non-blocking pulse engine

Code RoboBall cũ ghi GPIO bằng thanh ghi nhưng phát STEP bằng vòng lặp + delayMicroseconds(). TungLam_Stepper chuyển toàn bộ STEP timing sang Timer1.

Application không cần gọi run():

```cpp
ps2.update();
robot.update();
// sensor/state machine vẫn chạy

axis.moveTo(...); // Timer1 tiếp tục motion ở nền
```

Timer1 dùng prescaler /8. Trên Mega2560 16 MHz:

```text
Timer clock = 2 MHz
1 tick      = 0.5 us
```

Compare A thức dậy ở STEP event gần nhất. Interval dài được chia thành schedule chunk để vượt giới hạn 16-bit của OCR1A. Compare B chỉ làm nhiệm vụ kết thúc pulse STEP.

## 3. Hot path

TLFastPin dịch Arduino pin một lần thành:

- output register pointer;
- input register pointer;
- bit mask.

ISR dùng trực tiếp register. Không dùng digitalWrite() ở đường pulse.

STEP/DIR baseline D22..D29 nằm chung PORTA để shield/layout gọn và mở đường cho tối ưu mask theo nhóm sau này, nhưng pin không bị hard-code.

## 4. Driver abstraction

Motion engine chỉ phụ thuộc giao diện STEP/DIR/ENA:

- STEP pulse width;
- STEP active polarity;
- DIR setup time;
- EN polarity;
- DIR inversion.

Timing microsecond được chuyển sang timer tick khi begin()/setDriverConfig(). ISR chỉ đọc pulseWidthTicks_/directionSetupTicks_, tránh phép chia 64-bit trong đường nóng.

`stepActiveHigh=true` dùng pulse HIGH truyền thống; `false` giữ STEP idle HIGH và tạo pulse LOW cho wiring sinking/common-anode.

Microstep là một phần của motion model:

- Manual/DIP: TB6600, DM542, TMC STEP/DIR...
- A4988: adapter MS1/MS2/MS3 tùy chọn.
- DRV8825: adapter MODE0/MODE1/MODE2 tùy chọn.

Driver UART/SPI sau này có thể thêm adapter mà không thay planner.

## 5. Motion planner

Position move được lập kế hoạch ngoài ISR:

```text
Acceleration -> Cruise -> Deceleration
```

Move ngắn tự chuyển thành triangular profile.

AVR446-inspired interval recurrence dùng Q24.8 fixed-point để giữ phần lẻ sub-tick. Điều này tránh tình trạng phép chia integer về 0 sớm và acceleration bị kẹt ở tốc độ trung gian.

Float chỉ dùng ở thời điểm lập kế hoạch/chuyển đổi đơn vị, không dùng cho recurrence từng STEP.

## 6. Multi-axis scheduler

Mặc định:

```cpp
#define TUNGLAM_STEPPER_MAX_AXES 4
```

Các axis chạy độc lập trên cùng Timer1.

Khi application start/stop một axis trong lúc axis khác đang chạy, scheduler gọi synchronizeNow() để trừ chính xác thời gian đã trôi khỏi remaining interval trước khi re-arm compare. Việc thêm một axis mới vì vậy không kéo dài nhịp của axis đang chạy.

V0.1 chưa có synchronized-arrival group planner. Đây là tính năng khác với việc nhiều axis chạy đồng thời và được giữ cho roadmap sau.

## 7. Stop semantics

emergencyStop():

- ngắt schedule ngay;
- không tạo profile giảm tốc.

stop():

- position/continuous: tạo quãng giảm tốc;
- idempotent: gọi lặp lại không tính lại trajectory;
- position move không được mở rộng quá target cũ;
- soft-limit tiếp tục giới hạn stop envelope;
- homing chuyển sang emergency stop để không làm sai state machine.

## 8. Limit và homing

Hard limit chỉ chặn hướng đang đi vào switch. Hướng rời switch vẫn được phép.

Soft limits:

- kiểm tra trước command;
- kiểm tra trước từng STEP bình thường;
- 32-bit min/max được cập nhật atomically.

Homing cố ý bỏ qua soft-limit vì absolute position chưa đáng tin trước khi reference.

State machine:

```text
SeekFast
  -> confirm active N samples (không phát STEP sâu hơn trong lúc confirm)
  -> Backoff
  -> confirm released N samples
  -> extra backoff pulses
  -> SeekSlow
  -> confirm active N samples
  -> set home position
```

maxPhaseSteps bảo vệ từng pha homing. Nếu switch bị hỏng hoặc không bao giờ tác động, motion dừng với HomingTravelExceeded.

## 9. Position safety

Position dùng int32_t.

Các đường relative/absolute/unit conversion kiểm tra overflow. ISR cũng kiểm tra biên INT32_MIN/INT32_MAX trước STEP tiếp theo, đặc biệt quan trọng cho continuous/homing không có target hữu hạn.

## 10. Timer1 ownership

Library không được phép âm thầm phá Timer1 của subsystem khác.

begin() chỉ claim Timer1 khi:

- register vẫn đúng Arduino AVR core default: phase-correct 8-bit PWM + prescaler 64; hoặc
- timer hoàn toàn unused.

Nếu interrupt/mode/output đã bị thay đổi, begin() fail với Timer1Conflict.

Trước khi claim, library lưu:

- TCCR1A
- TCCR1B
- TIMSK1
- TCNT1
- OCR1A
- OCR1B

Khi axis cuối cùng gọi end(), register Timer1 được restore.

## 11. Resource contract của TungLam stack

```text
Timer0  Arduino millis()/micros()
Timer1  TungLam_Stepper
Timer2  auxiliary PWM
Timer3  TungLam_OmniMecanum_4WD
Timer4  TungLam_OmniMecanum_4WD
Timer5  preferred Servo pool
SPI     TungLam_PS2
```

Không gọi legacy TungLam_Control_MotorV5::Init_Timer1() khi Stepper đang dùng Timer1.

## 12. Diagnostics

printState(Print&) chụp atomically state chính:

- mode
- running
- stopping
- enabled
- position
- target
- interval
- fault
- homed
- homing state.

faultName(), modeName() và homingStateName() trả chuỗi F()/PROGMEM để giảm SRAM.
