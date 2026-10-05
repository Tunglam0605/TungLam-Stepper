# Driver-specific examples

Các sketch trong thư mục này là **reference examples theo từng driver**. Chúng được giữ trong `extras/` để không làm menu **File > Examples** của Arduino IDE bị quá dài.

## Danh sách

| Driver | Sketch | Microstep | STEP polarity minh họa | EN |
|---|---|---|---|---|
| A4988 | `A4988/A4988.ino` | MS1/MS2/MS3 do library điều khiển | active HIGH | nEN active LOW |
| DRV8825 | `DRV8825/DRV8825.ino` | MODE0/1/2 do library điều khiển | active HIGH | nEN active LOW |
| TB6600 | `TB6600/TB6600.ino` | DIP/manual | active HIGH, common-cathode example | bỏ qua trong ví dụ |
| DM542 | `DM542/DM542.ino` | DIP/manual | active LOW, sinking/common-anode example | bỏ qua trong ví dụ |
| TMC2209 | `TMC2209_STEPDIR/TMC2209_STEPDIR.ino` | external/manual/UART bên ngoài library | active HIGH | active LOW trên carrier phổ biến |

## Quy ước pin Mega2560

Các ví dụ dùng cùng baseline để đổi driver mà không phải đổi kiến trúc project:

```text
STEP = D22
DIR  = D23
EN   = D42  (nếu dùng)
MS1/MODE0 = D46
MS2/MODE1 = D47
MS3/MODE2 = D48
```

## A4988 / DRV8825

Hai driver này có profile microstep tích hợp:

```cpp
axis.attachMicrostepPins(MS1, MS2, MS3);

axis.setMicrostepMode(
    16,
    TungLamMicrostepDriverProfile::A4988
);
```

hoặc:

```cpp
axis.setMicrostepMode(
    32,
    TungLamMicrostepDriverProfile::DRV8825
);
```

Library tự ghi truth-table của ba chân MS/MODE.

## TB6600 / DM542

Với các driver công nghiệp/DIP:

```cpp
axis.setMicrosteps(16);
```

chỉ **khai báo cho model cơ khí biết hệ số vi bước thực tế**. Library không thể thay đổi DIP vật lý.

### STEP active HIGH

Ví dụ common-cathode / drive HIGH:

```cpp
TungLamStepperDriverConfig driver(
    true,
    false,
    6,
    6,
    true
);
```

### STEP active LOW / sinking

Ví dụ common-anode:

```cpp
TungLamStepperDriverConfig driver(
    true,
    false,
    6,
    6,
    false
);
```

Khi `stepActiveHigh=false`:

```text
STEP idle  = HIGH
STEP pulse = LOW
```

## TMC2209

Ví dụ hiện tại chỉ dùng **STEP/DIR mode**.

`TungLam_Stepper` không cấu hình:

- UART address;
- current;
- stealthChop/spreadCycle;
- interpolation;
- microstep register của TMC2209.

Các phần đó phải do subsystem UART/TMC riêng xử lý. Sau đó chỉ cần khai báo đúng microstep thực tế cho motion model:

```cpp
axis.setMicrosteps(16);
```

## Lưu ý wiring quan trọng

Tên driver giống nhau nhưng module/carrier của các hãng khác nhau có thể khác:

- polarity ENA;
- cách mắc optocoupler;
- điện áp logic/input;
- điện trở hạn dòng tích hợp;
- nhãn PUL/STEP, CW/DIR, ENA/EN.

Vì vậy ví dụ là **baseline phần mềm**, không thay thế datasheet/sơ đồ chân của đúng module đang cầm trên tay.

Đặc biệt với TB6600/DM542 và input optocoupler, không mặc định rằng một chân Arduino có thể source/sink trực tiếp mọi module. Hãy kiểm tra yêu cầu điện áp/dòng input của driver cụ thể và dùng tầng giao tiếp phù hợp khi cần.

## Chọn example nào?

```text
A4988
  -> A4988/A4988.ino

DRV8825
  -> DRV8825/DRV8825.ino

TB6600 DIP
  -> TB6600/TB6600.ino

DM542 / optocoupler sinking
  -> DM542/DM542.ino

TMC2209 STEP/DIR
  -> TMC2209_STEPDIR/TMC2209_STEPDIR.ino

Driver STEP/DIR khác
  -> lấy TB6600 hoặc DM542 làm baseline,
     chỉnh pulseWidthUs / directionSetupUs / polarity theo datasheet.
```
