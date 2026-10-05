# Architecture

## 1. Layering

```text
Application
  ├─ PS2 / buttons / Serial / ROS2 bridge / state machine
  │
  └─ TungLamStepper public API
         │
         ├─ Mechanism model
         │    full-step/rev
         │    microsteps
         │    gear ratio
         │    travel/rev
         │
         ├─ Motion planner
         │    target position
         │    trapezoidal acceleration
         │    controlled stop
         │
         ├─ Safety
         │    MIN/MAX hard limits
         │    software limits
         │    emergency stop
         │
         └─ Timer1 event engine
              ├─ Compare A: next STEP rising event
              └─ Compare B: STEP pulse falling event
```

## 2. Không block loop

Code RoboBall cũ phát xung bằng vòng `for + delayMicroseconds()`. Cách đó giữ CPU cho tới khi chạy đủ số pulse.

TungLam_Stepper chuyển pulse generation sang Timer1. Application không phải gọi `run()` để duy trì bước.

## 3. Fast pin abstraction

Pin do người dùng chọn vẫn được dịch một lần thành:

- output-register pointer;
- input-register pointer;
- bit mask.

Sau đó STEP/DIR/ENA/limit dùng trực tiếp register. Pin baseline trên cùng PORT giúp đường tối ưu sau này có thể gom nhiều bit trong một lần ghi.

## 4. Driver abstraction

Motion engine chỉ biết STEP/DIR/ENA và timing:

- pulse width;
- direction setup;
- enable polarity;
- direction inversion.

Microstep là thông số của **motion model**, không bắt buộc là GPIO của library. Vì vậy cùng core dùng được với:

- A4988 / DRV8825: microstep có thể đặt bằng MS pins ngoài core;
- TB6600 / DM542: microstep đặt bằng DIP;
- TMC: sau này thêm adapter UART/SPI mà không đổi planner.

## 5. Timer scheduler

Timer1 chạy prescaler /8. Với Mega2560 16 MHz:

```text
Timer clock = 2 MHz
1 tick      = 0.5 us
```

Compare A chỉ thức dậy tại step event gần nhất; interval dài được chia thành schedule chunk. Compare B kết thúc pulse STEP theo `pulseWidthUs`.

Thiết kế này tránh fixed high-frequency tick ISR và tránh `delayMicroseconds()`.

## 6. Multi-axis

Mặc định tối đa 4 axis:

```cpp
#define TUNGLAM_STEPPER_MAX_AXES 4
```

Có thể override macro trước compile nếu platform/tài nguyên cho phép. V0.1 chạy các axis độc lập chung timer. Synchronized-arrival group planner là milestone tiếp theo, không trộn vào single-axis core.

## 7. Safety semantics

- Hard limit chỉ chặn chuyển động **đi vào** limit; vẫn cho phép chạy ra khỏi limit.
- Soft limit được kiểm tra trước lệnh và trước từng pulse.
- `emergencyStop()`: dừng phát pulse ngay.
- `stop()`: với position move, chuyển sang quãng giảm tốc tính từ tốc độ hiện tại.
- Driver không tự disable sau move để giữ torque; user có thể gọi `disable()`.

## 8. Compatibility with existing TungLam libraries

```text
Timer0  Arduino timebase
Timer1  TungLam_Stepper
Timer2  auxiliary PWM
Timer3  TungLam_OmniMecanum_4WD
Timer4  TungLam_OmniMecanum_4WD
Timer5  preferred Servo pool
SPI     TungLam_PS2
```

Modern Drive API phù hợp với resource contract này. Legacy `Init_Timer1()` không được gọi khi dùng Stepper.
