# TungLam Stepper — Mega2560 Resource & Pin Baseline

## Mục tiêu

Pinout này được chốt để ba thư viện có thể cùng tồn tại trên một Arduino Mega2560 mà không tranh chấp tài nguyên chính:

- `TungLam_PS2`
- `TungLam_OmniMecanum_4WD`
- `TungLam_Stepper`

## Resource map

| Module | Resource | Pins |
|---|---|---|
| TungLam_OmniMecanum_4WD | Timer3 + Timer4 | PWM D5..D8, DIR D30..D37 |
| TungLam_Stepper | Timer1 | STEP/DIR baseline D22..D29 |
| TungLam_PS2 | Hardware SPI | D50..D53 |
| Stepper enable | GPIO | D42..D45 |
| Stepper limits | PORTK | A8..A15 |
| Servo | ưu tiên Timer5 | pin signal tùy ứng dụng |
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

STEP/DIR cùng PORTA giúp layout shield gọn và cho phép tối ưu bit-mask sau này. Limit cùng PORTK giúp việc đọc trạng thái nhiều limit rất nhanh và cho phép nâng cấp sang PCINT nếu cần.

## Timer1 contract

`TungLam_Stepper::begin()` chủ động lấy Timer1 khỏi cấu hình PWM mặc định của Arduino AVR core.

Vì vậy khi Stepper đã begin:

- không dùng `analogWrite()` trên các output Timer1 D11/D12;
- không gọi legacy `TungLam_Control_MotorV5::Init_Timer1()`;
- không dùng thư viện khác đã chiếm Timer1 interrupt;
- Servo trên Mega nên giữ trong nhóm Timer5 trước, tránh mở rộng tới mức Servo library phải dùng thêm Timer1.

## Pin flexibility

D22..D29/D42..D45/A8..A15 là **baseline tối ưu**, không phải chân bắt buộc.

Constructor vẫn nhận pin tùy ý:

```cpp
TungLamStepper axis(stepPin, dirPin, enablePin);
axis.attachLimits(minPin, maxPin);
```

Trên AVR, library cache port register + bit mask một lần trong `begin()`; đường chạy STEP không dùng `digitalWrite()`.
