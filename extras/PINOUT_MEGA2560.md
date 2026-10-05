# TungLam Stepper — Mega2560 Resource & Pin Baseline

## Mục tiêu

Pinout này được chốt để ba thư viện cùng tồn tại trên Arduino Mega2560 mà không tranh chấp tài nguyên chính:

- TungLam_PS2
- TungLam_OmniMecanum_4WD
- TungLam_Stepper

Đây là **baseline khuyến nghị**, không phải chân bắt buộc.

## Resource map

| Module | Resource | Pins |
|---|---|---|
| TungLam_OmniMecanum_4WD | Timer3 + Timer4 | PWM D5..D8, DIR D30..D37 |
| TungLam_Stepper | Timer1 | STEP/DIR baseline D22..D29 |
| TungLam_PS2 | Hardware SPI | D50..D53 |
| Stepper enable | GPIO | D42..D45 |
| Stepper limits | PORTK | A8..A15 |
| Optional MS pins | GPIO | ví dụ D46..D48 cho một driver |
| Servo | ưu tiên Timer5 | signal tùy ứng dụng |
| Aux PWM | Timer2 | D9, D10 |
| I2C | TWI | D20, D21 |
| Serial1 | UART | D18, D19 |
| Serial2 | UART | D16, D17 |
| Serial3 | UART | D14, D15 |

## Axis baseline

| Axis | STEP | DIR | EN | MIN/HOME | MAX |
|---|---:|---:|---:|---:|---:|
| 1 | D22 / PA0 | D23 / PA1 | D42 / PL7 | A8 / PK0 | A9 / PK1 |
| 2 | D24 / PA2 | D25 / PA3 | D43 / PL6 | A10 / PK2 | A11 / PK3 |
| 3 | D26 / PA4 | D27 / PA5 | D44 / PL5 | A12 / PK4 | A13 / PK5 |
| 4 | D28 / PA6 | D29 / PA7 | D45 / PL4 | A14 / PK6 | A15 / PK7 |

STEP/DIR cùng PORTA giúp layout shield gọn. Limit cùng PORTK giúp routing đồng nhất và đọc register nhanh.

## Timer1 contract

Arduino AVR core mặc định cấu hình Timer1 ở phase-correct 8-bit PWM với prescaler 64.

TungLam_Stepper chỉ claim Timer1 khi timer vẫn đúng trạng thái mặc định này hoặc hoàn toàn unused. Nếu subsystem khác đã đổi Timer1 mode/interrupt/output, begin() trả false với Timer1Conflict.

Library lưu register trước khi claim và khôi phục khi axis Stepper cuối cùng gọi end().

Trong thời gian Stepper sở hữu Timer1:

- không gọi analogWrite() trên Timer1 outputs D11/D12;
- không gọi legacy TungLam_Control_MotorV5::Init_Timer1();
- không dùng thư viện khác cần Timer1 interrupt;
- Servo trên Mega nên nằm trong Timer5 trước, tránh mở rộng tới Timer1.

## Optional microstep pins

Microstep pins không nằm trong baseline bắt buộc vì nhiều driver công nghiệp dùng DIP.

Với A4988/DRV8825 có thể chọn GPIO còn trống, ví dụ cho một axis:

```text
MS1/MODE0 -> D46
MS2/MODE1 -> D47
MS3/MODE2 -> D48
```

Đây chỉ là ví dụ wiring; API cho phép pin khác.

## Pin flexibility

```cpp
TungLamStepper axis(stepPin, dirPin, enablePin);
axis.attachLimits(minPin, maxPin);
```

Trong begin()/attach, library cache port register và bit mask. Đường STEP ISR không dùng digitalWrite().
