/*==============================================================================
  TUNGLAM STEPPER
  ==============================================================================
  Non-blocking STEP/DIR motion engine for Arduino AVR robots and mechanisms.

  Design goals:
  - Timer-driven pulse generation: no delay()/delayMicroseconds() pulse loops.
  - Direct cached port access on AVR for fast STEP/DIR/ENA/limit I/O.
  - Generic STEP/DIR drivers: A4988, DRV8825, TB6600, DM542 and similar.
  - Driver-independent motion model: full steps, microsteps, gear ratio, travel.
  - Up to TUNGLAM_STEPPER_MAX_AXES independent axes (default 4).
  - Position moves with trapezoidal acceleration/deceleration.
  - Continuous constant-speed mode.
  - Hard limit and software-limit protection.
  - Coexists with TungLam_OmniMecanum_4WD when Timer1 is reserved for Stepper.

  Recommended Arduino Mega 2560 baseline:
    Axis 1: STEP D22, DIR D23, EN D42, MIN A8,  MAX A9
    Axis 2: STEP D24, DIR D25, EN D43, MIN A10, MAX A11
    Axis 3: STEP D26, DIR D27, EN D44, MIN A12, MAX A13
    Axis 4: STEP D28, DIR D29, EN D45, MIN A14, MAX A15

  Resource contract on Mega:
    Timer1 -> TungLam_Stepper
    Timer3 + Timer4 -> TungLam_OmniMecanum_4WD
    SPI D50..D53 -> TungLam_PS2
==============================================================================*/

#pragma once

#include <Arduino.h>
#include "internal/TLFastPin.h"

namespace tunglam {
namespace stepper {
namespace internal {
class TLTimerEngine;
}
}
}

/**
 * @brief Mã lỗi runtime gần nhất của một axis.
 *
 * Dùng fault() để đọc, faultName() để in tên dễ hiểu và clearFault() để xóa
 * sau khi nguyên nhân lỗi đã được xử lý.
 */
enum class TungLamStepperFault : uint8_t {
  None = 0,              ///< Không có lỗi.
  InvalidPin,            ///< Chân cấu hình không hợp lệ/không ánh xạ được.
  NoAxisSlot,            ///< Scheduler đã dùng hết số slot axis cho phép.
  Timer1Conflict,        ///< Timer1 đang bị subsystem khác sở hữu/cấu hình.
  InvalidConfig,         ///< Thông số timing/motion/homing không hợp lệ.
  SoftLimit,             ///< Lệnh hoặc STEP tiếp theo vượt giới hạn mềm.
  MinLimit,              ///< Đang đi vào công tắc MIN đang tác động.
  MaxLimit,              ///< Đang đi vào công tắc MAX đang tác động.
  PositionOverflow,      ///< Position/move vượt miền int32 hoặc planner an toàn.
  HomingTravelExceeded   ///< Một pha homing vượt maxPhaseSteps.
};

/**
 * @brief Chế độ vận hành hiện tại của axis.
 */
enum class TungLamStepperMode : uint8_t {
  Idle = 0,    ///< Đang đứng yên, không phát STEP.
  Position,    ///< Đang chạy tới target tuyệt đối/tương đối có ramp.
  Continuous,  ///< Đang chạy liên tục ở tốc độ cố định.
  Homing       ///< Đang chạy state machine tìm gốc.
};

/**
 * @brief Pha hiện tại của state machine homing.
 */
enum class TungLamStepperHomingState : uint8_t {
  Idle = 0,  ///< Chưa homing hoặc homing đã bị hủy.
  SeekFast,  ///< Đi nhanh về phía HOME để tìm switch lần đầu.
  Backoff,   ///< Rời khỏi switch và lùi thêm backoffSteps.
  SeekSlow,  ///< Tiến chậm lại để bắt cạnh switch chính xác hơn.
  Complete   ///< Homing thành công và position đã được gán homePositionSteps.
};

/**
 * @brief Chiều chuyển động logic của trục.
 *
 * Positive/Negative là chiều tọa độ phần mềm. Nếu hướng cơ khí ngược mong muốn,
 * dùng directionInverted trong TungLamStepperDriverConfig thay vì đổi toàn bộ code.
 */
enum class TungLamStepperDirection : int8_t {
  Negative = -1,  ///< Chiều âm của trục tọa độ.
  Positive = 1    ///< Chiều dương của trục tọa độ.
};

/**
 * @brief Profile truth-table cho chân chọn vi bước.
 */
enum class TungLamMicrostepDriverProfile : uint8_t {
  Manual = 0,  ///< Library chỉ lưu hệ số microstep, không tự điều khiển MS pins.
  A4988,       ///< Dùng truth-table MS1/MS2/MS3 của A4988.
  DRV8825      ///< Dùng truth-table MODE0/MODE1/MODE2 của DRV8825.
};

/**
 * @brief Timing và polarity của driver STEP/DIR/ENA.
 *
 * Constructor giữ tương thích source với bản 4 tham số cũ; stepHigh là tham số
 * cuối tùy chọn để hỗ trợ cả pulse active-HIGH và common-anode/sinking active-LOW.
 */
struct TungLamStepperDriverConfig {
  bool enableActiveLow;       ///< true nếu ENA được kích bằng mức LOW.
  bool directionInverted;     ///< true để đảo ý nghĩa logic DIR.
  uint16_t pulseWidthUs;      ///< Độ rộng mức active của STEP [us].
  uint16_t directionSetupUs;  ///< Thời gian DIR ổn định trước STEP đầu [us].
  bool stepActiveHigh;        ///< true: idle LOW/pulse HIGH; false: idle HIGH/pulse LOW.

  /**
   * @brief Tạo cấu hình timing/polarity cho driver STEP/DIR.
   * @param enActiveLow true nếu ENA active LOW; false nếu active HIGH.
   * @param dirInverted true để đảo chiều logic DIR của toàn axis.
   * @param pulseUs Độ rộng mức active của STEP [us], mặc định 4 us.
   * @param dirSetup Thời gian DIR ổn định trước STEP đầu [us], mặc định 4 us.
   * @param stepHigh true = pulse HIGH; false = pulse LOW/sinking.
   */
  TungLamStepperDriverConfig(bool enActiveLow = true,
                             bool dirInverted = false,
                             uint16_t pulseUs = 4,
                             uint16_t dirSetup = 4,
                             bool stepHigh = true)
      : enableActiveLow(enActiveLow),
        directionInverted(dirInverted),
        pulseWidthUs(pulseUs),
        directionSetupUs(dirSetup),
        stepActiveHigh(stepHigh) {}
};

/**
 * @brief Cấu hình homing non-blocking hai lượt.
 */
struct TungLamStepperHomingConfig {
  TungLamStepperDirection direction;       ///< Hướng đi tìm home.
  uint32_t fastSpeedStepsPerSecond;        ///< Tốc độ seek nhanh [pulse/s].
  uint32_t slowSpeedStepsPerSecond;        ///< Tốc độ bắt lại chậm [pulse/s].
  uint32_t backoffSteps;                   ///< Pulse đi thêm sau khi xác nhận switch đã nhả.
  int32_t homePositionSteps;               ///< Position gán sau homing thành công.
  uint8_t confirmSamples;                  ///< Số mẫu liên tiếp để xác nhận active/released.
  uint32_t maxPhaseSteps;                  ///< Giới hạn pulse mỗi pha; 0 = không giới hạn.

  /**
   * @brief Tạo cấu hình homing hai lượt.
   * @param homeDirection Hướng cơ cấu chạy tới công tắc HOME.
   * @param fastSpeed Tốc độ seek lần đầu [pulse/s].
   * @param slowSpeed Tốc độ bắt lại switch lần hai [pulse/s].
   * @param backoff Số pulse đi thêm sau khi switch đã nhả.
   * @param homePosition Giá trị position gán khi homing thành công [pulse].
   * @param debounceSamples Số lần đọc liên tiếp để xác nhận switch.
   * @param maxTravelPerPhase Số pulse tối đa cho mỗi pha; 0 = không giới hạn.
   */
  TungLamStepperHomingConfig(
      TungLamStepperDirection homeDirection = TungLamStepperDirection::Negative,
      uint32_t fastSpeed = 2000,
      uint32_t slowSpeed = 400,
      uint32_t backoff = 100,
      int32_t homePosition = 0,
      uint8_t debounceSamples = 3,
      uint32_t maxTravelPerPhase = 0)
      : direction(homeDirection),
        fastSpeedStepsPerSecond(fastSpeed),
        slowSpeedStepsPerSecond(slowSpeed),
        backoffSteps(backoff),
        homePositionSteps(homePosition),
        confirmSamples(debounceSamples),
        maxPhaseSteps(maxTravelPerPhase) {}
};

/**
 * @brief Giới hạn motion theo đơn vị pulse.
 */
struct TungLamStepperMotionConfig {
  uint32_t maxSpeedStepsPerSecond;         ///< Tốc độ cực đại [pulse/s].
  uint32_t accelerationStepsPerSecond2;    ///< Gia tốc [pulse/s^2].
  uint32_t decelerationStepsPerSecond2;    ///< Giảm tốc [pulse/s^2].

  /**
   * @brief Tạo profile giới hạn tốc độ/gia tốc cơ bản.
   * @param maxSpeed Tốc độ STEP cực đại [pulse/s].
   * @param acceleration Gia tốc khi tăng tốc [pulse/s^2].
   * @param deceleration Gia tốc giảm tốc [pulse/s^2].
   */
  TungLamStepperMotionConfig(uint32_t maxSpeed = 4000,
                             uint32_t acceleration = 8000,
                             uint32_t deceleration = 8000)
      : maxSpeedStepsPerSecond(maxSpeed),
        accelerationStepsPerSecond2(acceleration),
        decelerationStepsPerSecond2(deceleration) {}
};

/**
 * @brief Bộ điều khiển một trục STEP/DIR chạy nền bằng Timer1.
 *
 * Sau begin(), xung STEP được phát trong ISR; ứng dụng không cần gọi run().
 * Các đối tượng chia sẻ cùng một Timer1 scheduler và mặc định tối đa 4 trục.
 */
class TungLamStepper {
 public:
  /** @brief Giá trị dùng để báo một chân tùy chọn không được sử dụng. */
  static constexpr uint8_t NO_PIN = 0xFF;

  /**
   * @brief Tạo một trục stepper.
   * @param stepPin Chân STEP/PUL.
   * @param dirPin Chân DIR.
   * @param enablePin Chân ENA/ENABLE; bỏ qua bằng NO_PIN nếu driver không dùng.
   */
  TungLamStepper(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin = NO_PIN);
  ~TungLamStepper();

  /**
   * @brief Khởi tạo với cấu hình driver/motion mặc định.
   * @return true khi lấy được Timer1 và đăng ký axis; false nếu pin/config/timer xung đột.
   */
  bool begin();

  /**
   * @brief Khởi tạo với timing/polarity và giới hạn motion tùy chỉnh.
   * @param driver Cấu hình STEP/DIR/ENA và timing của driver.
   * @param motion Giới hạn tốc độ, gia tốc và giảm tốc của planner.
   * @return true nếu cấu hình hợp lệ và Timer1 có thể được sở hữu an toàn.
   */
  bool begin(const TungLamStepperDriverConfig& driver,
             const TungLamStepperMotionConfig& motion);

  /**
   * @brief Gỡ axis khỏi scheduler và khôi phục Timer1 khi đây là axis cuối.
   * @param disableDriver true để đưa ENA về trạng thái disable trước khi trả timer.
   */
  void end(bool disableDriver = true);

  /** @brief Kích hoạt driver theo polarity đã cấu hình. */
  void enable();

  /** @brief Dừng ngay chuyển động hiện tại rồi disable driver. */
  void disable();

  /**
   * @brief Kiểm tra trạng thái enable logic mà library đang quản lý.
   * @return true nếu driver đang được coi là enabled.
   */
  bool enabled() const;

  /**
   * @brief Bật/tắt tự enable driver khi bắt đầu một lệnh motion.
   * @param enabled true để tự enable trước motion; false để application tự quản lý ENA.
   * @note Mặc định true; kết thúc move không tự disable để giữ mô-men.
   */
  void setAutoEnable(bool enabled);

  /**
   * @brief Đọc chế độ auto-enable hiện tại.
   * @return true nếu driver sẽ tự enable trước mỗi lệnh motion.
   */
  bool autoEnable() const;

  /**
   * @brief Cập nhật timing/polarity của driver khi axis đang idle.
   * @param config Cấu hình STEP/DIR/ENA mới.
   * @return true nếu cấu hình hợp lệ và tương thích maxSpeed hiện tại.
   * @note Không đổi cấu hình driver khi axis đang chạy.
   */
  bool setDriverConfig(const TungLamStepperDriverConfig& config);

  /**
   * @brief Đọc cấu hình STEP/DIR/ENA hiện tại.
   * @return Tham chiếu read-only tới TungLamStepperDriverConfig đang dùng.
   */
  const TungLamStepperDriverConfig& driverConfig() const;

  /**
   * @brief Cập nhật giới hạn tốc độ/gia tốc của planner khi axis đang idle.
   * @param config Profile motion mới.
   * @return true nếu profile hợp lệ và không vượt trần timing của driver.
   */
  bool setMotionConfig(const TungLamStepperMotionConfig& config);

  /**
   * @brief Đọc profile tốc độ/gia tốc hiện tại.
   * @return Tham chiếu read-only tới TungLamStepperMotionConfig đang dùng.
   */
  const TungLamStepperMotionConfig& motionConfig() const;

  /**
   * @brief Đặt tốc độ STEP cực đại cho position move.
   * @param stepsPerSecond Tốc độ [pulse/s].
   * @return true nếu giá trị hợp lệ và không vượt trần timing hiện tại.
   */
  bool setMaxSpeed(uint32_t stepsPerSecond);

  /**
   * @brief Đặt gia tốc khi tăng tốc.
   * @param stepsPerSecond2 Gia tốc [pulse/s^2].
   * @return true nếu profile sau cập nhật hợp lệ.
   */
  bool setAcceleration(uint32_t stepsPerSecond2);

  /**
   * @brief Đặt gia tốc giảm tốc.
   * @param stepsPerSecond2 Giảm tốc [pulse/s^2].
   * @return true nếu profile sau cập nhật hợp lệ.
   */
  bool setDeceleration(uint32_t stepsPerSecond2);

  /** @brief Khai báo số full-step của motor cho một vòng rotor, ví dụ 200 với motor 1.8°. */
  /**
   * @brief Khai báo số full-step của motor cho một vòng rotor.
   * @param fullSteps Ví dụ 200 với motor 1.8° hoặc 400 với motor 0.9°.
   * @return true nếu >0 và axis đang idle.
   */
  bool setMotorFullStepsPerRevolution(uint16_t fullSteps);

  /**
   * @brief Khai báo hệ số vi bước đang được driver sử dụng.
   * @note Dùng trực tiếp cho TB6600/DM542 đặt microstep bằng DIP.
   */
  /**
   * @brief Khai báo hệ số vi bước đang được driver sử dụng.
   * @param microsteps Hệ số 1, 2, 4, 8, 16... tối đa 1024.
   * @return true nếu hợp lệ và axis đang idle.
   * @note Dùng trực tiếp cho TB6600/DM542 đặt microstep bằng DIP.
   */
  bool setMicrosteps(uint16_t microsteps);

  /**
   * @brief Gắn ba chân MS/MODE tùy chọn cho A4988/DRV8825.
   * @param ms1Pin Chân MS1/MODE0.
   * @param ms2Pin Chân MS2/MODE1.
   * @param ms3Pin Chân MS3/MODE2.
   * @return true nếu cả ba chân hợp lệ và axis đang idle.
   * @note Không bắt buộc; core motion vẫn hoạt động với driver đặt microstep bằng DIP.
   */
  bool attachMicrostepPins(uint8_t ms1Pin, uint8_t ms2Pin, uint8_t ms3Pin);

  /** @brief Gỡ quyền điều khiển ba chân MS/MODE khỏi axis. */
  void detachMicrostepPins();

  /**
   * @brief Kiểm tra library có đang quản lý ba chân chọn microstep hay không.
   * @return true nếu đủ ba chân MS1/MS2/MS3 đã được attach.
   */
  bool microstepPinsAttached() const;

  /** @brief Ghi trực tiếp ba mức logic MS1/MS2/MS3 theo nhu cầu custom. */
  /**
   * @brief Ghi trực tiếp mức logic lên ba chân MS/MODE.
   * @param ms1 Mức logic chân MS1/MODE0.
   * @param ms2 Mức logic chân MS2/MODE1.
   * @param ms3 Mức logic chân MS3/MODE2.
   * @return true nếu MS pins đã attach và axis đang idle.
   */
  bool setMicrostepPinLevels(bool ms1, bool ms2, bool ms3);

  /**
   * @brief Chọn vi bước bằng truth-table tích hợp của A4988 hoặc DRV8825.
   * @return false nếu profile/mức vi bước không hợp lệ hoặc chưa attach MS pins.
   */
  /**
   * @brief Chọn microstep và tự áp truth-table của driver.
   * @param microsteps Hệ số vi bước mong muốn.
   * @param profile A4988 hoặc DRV8825.
   * @return true nếu profile/hệ số hợp lệ, MS pins đã attach và axis idle.
   */
  bool setMicrostepMode(uint16_t microsteps,
                        TungLamMicrostepDriverProfile profile);

  /**
   * @brief Đặt tỷ số truyền motor/output.
   * @param motorRevolutionsPerOutputRevolution Số vòng motor cho một vòng đầu ra.
   */
  /**
   * @brief Đặt tỷ số truyền motor/output.
   * @param motorRevolutionsPerOutputRevolution Số vòng motor cần cho 1 vòng output.
   *        Ví dụ hộp số 5:1 dùng giá trị 5.0.
   * @return true nếu >0 và axis đang idle.
   */
  bool setGearRatio(float motorRevolutionsPerOutputRevolution);

  /**
   * @brief Đặt hành trình tuyến tính của một vòng đầu ra.
   * @param travelUnits Có thể là mm/rev, cm/rev... miễn toàn project dùng cùng đơn vị.
   * @return true nếu giá trị >0 và axis đang idle.
   */
  bool setTravelPerOutputRevolution(float travelUnits);

  /**
   * @brief Đọc số full-step trên một vòng rotor.
   * @return Giá trị full-step/rev hiện tại.
   */
  uint16_t motorFullStepsPerRevolution() const;

  /**
   * @brief Đọc hệ số vi bước hiện tại.
   * @return Hệ số microstep đang dùng.
   */
  uint16_t microsteps() const;

  /**
   * @brief Đọc tỷ số truyền motor/output hiện tại.
   * @return Số vòng motor trên một vòng output.
   */
  float gearRatio() const;

  /**
   * @brief Đọc hành trình tuyến tính của một vòng output.
   * @return Hành trình/rev theo đơn vị do application quy ước.
   */
  float travelPerOutputRevolution() const;

  /**
   * @brief Tính số xung STEP cần cho một vòng output.
   * @return fullStepsPerRev × microsteps × gearRatio [pulse/rev output].
   */
  float pulsesPerOutputRevolution() const;

  /**
   * @brief Di chuyển tương đối theo số pulse/step với profile tăng/giảm tốc.
   * @param relativeSteps Số pulse tương đối; dương đi Positive, âm đi Negative.
   * @return true nếu lệnh được nhận và planner bắt đầu/chấp nhận motion.
   */
  bool move(int32_t relativeSteps);

  /**
   * @brief Di chuyển tới position tuyệt đối theo pulse.
   * @param absolutePositionSteps Target position [pulse].
   * @return true nếu target hợp lệ và motion được lập kế hoạch.
   */
  bool moveTo(int32_t absolutePositionSteps);

  /**
   * @brief Di chuyển tương đối theo số vòng của trục đầu ra.
   * @param outputRevolutions Số vòng output tương đối.
   * @return true nếu quy đổi an toàn và motion được nhận.
   */
  bool moveRevolutions(float outputRevolutions);

  /**
   * @brief Di chuyển tương đối theo góc đầu ra.
   * @param outputDegrees Góc tương đối [độ].
   * @return true nếu quy đổi an toàn và motion được nhận.
   */
  bool moveDegrees(float outputDegrees);

  /**
   * @brief Di chuyển tương đối theo hành trình tuyến tính.
   * @param travelUnits Hành trình theo đơn vị đã khai báo trong travel/rev.
   * @return true nếu travel model hợp lệ và motion được nhận.
   */
  bool moveTravel(float travelUnits);

  /**
   * @brief Đi tới position tuyệt đối tính theo số vòng output.
   * @param outputRevolutions Position tuyệt đối [vòng output].
   * @return true nếu quy đổi an toàn và motion được nhận.
   */
  bool moveToRevolutions(float outputRevolutions);

  /**
   * @brief Đi tới position tuyệt đối tính theo góc output.
   * @param outputDegrees Position tuyệt đối [độ].
   * @return true nếu quy đổi an toàn và motion được nhận.
   */
  bool moveToDegrees(float outputDegrees);

  /**
   * @brief Đi tới position tuyệt đối theo hành trình tuyến tính.
   * @param travelUnits Position tuyệt đối theo đơn vị travel đã cấu hình.
   * @return true nếu travel model hợp lệ và motion được nhận.
   */
  bool moveToTravel(float travelUnits);

  /**
   * @brief Quy đổi số vòng output sang pulse.
   * @param outputRevolutions Số vòng output.
   * @return Số pulse tương ứng, saturate ở biên int32 nếu quá lớn.
   */
  int32_t revolutionsToSteps(float outputRevolutions) const;

  /**
   * @brief Quy đổi góc output sang pulse.
   * @param outputDegrees Góc [độ].
   * @return Số pulse tương ứng.
   */
  int32_t degreesToSteps(float outputDegrees) const;

  /**
   * @brief Quy đổi hành trình tuyến tính sang pulse.
   * @param travelUnits Hành trình theo cùng đơn vị đã dùng trong setTravelPerOutputRevolution().
   * @return Số pulse tương ứng; trả 0 nếu chưa cấu hình travel/rev hợp lệ.
   */
  int32_t travelToSteps(float travelUnits) const;

  /**
   * @brief Quy đổi pulse thành số vòng output.
   * @param steps Số pulse có dấu.
   * @return Số vòng output tương ứng.
   */
  float stepsToRevolutions(int32_t steps) const;

  /**
   * @brief Quy đổi pulse thành góc output.
   * @param steps Số pulse có dấu.
   * @return Góc output [độ].
   */
  float stepsToDegrees(int32_t steps) const;

  /**
   * @brief Quy đổi pulse thành hành trình tuyến tính.
   * @param steps Số pulse có dấu.
   * @return Hành trình theo đơn vị đã cấu hình; 0 nếu travel model chưa hợp lệ.
   */
  float stepsToTravel(int32_t steps) const;

  /**
   * @brief Chạy liên tục ở tốc độ pulse cố định.
   * @param direction Hướng Positive/Negative.
   * @param stepsPerSecond Tần số STEP [pulse/s], không được vượt maxSpeed đã cấu hình.
   * @return true nếu axis bắt đầu chạy; false nếu đang bận hoặc tốc độ không hợp lệ.
   * @note Đây là continuous constant-speed; position move mới dùng ramp tăng tốc đầy đủ.
   */
  bool runContinuous(TungLamStepperDirection direction,
                     uint32_t stepsPerSecond);

  /**
   * @brief Chạy homing non-blocking: seek nhanh -> backoff -> seek chậm -> set zero.
   * @param config Tốc độ, hướng, backoff, debounce và giới hạn hành trình từng pha.
   * @return false nếu thiếu limit phù hợp hoặc cấu hình không hợp lệ.
   */
  bool home(const TungLamStepperHomingConfig& config =
                TungLamStepperHomingConfig());

  /**
   * @brief Kiểm tra state machine homing có đang chạy hay không.
   * @return true khi axis đang ở Homing mode.
   */
  bool isHoming() const;

  /**
   * @brief Kiểm tra axis đã có mốc home hợp lệ hay chưa.
   * @return true sau khi homing hoàn tất thành công và chưa bị clearHomed().
   */
  bool isHomed() const;

  /**
   * @brief Đọc pha hiện tại của state machine homing.
   * @return Idle/SeekFast/Backoff/SeekSlow/Complete.
   */
  TungLamStepperHomingState homingState() const;

  /** @brief Xóa cờ homed khi application làm mất mốc cơ khí. */
  void clearHomed();

  /**
   * @brief Dừng có giảm tốc, không vượt target hiện tại/soft limit.
   * @note Nếu đang homing, stop() chuyển thành emergencyStop() để không làm sai state.
   */
  void stop();

  /** @brief Ngắt phát STEP ngay lập tức, không áp profile giảm tốc. */
  void emergencyStop();

  /**
   * @brief Kiểm tra axis có motion đang hoạt động hay không.
   * @return true nếu axis đang phát/đợi phát STEP.
   */
  bool isRunning() const;

  /**
   * @brief Kiểm tra axis có đang chạy tới target position hay không.
   * @return true nếu mode là Position và motion chưa hoàn tất.
   */
  bool isMovingToPosition() const;

  /**
   * @brief Kiểm tra axis đang ở pha controlled-stop hay không.
   * @return true sau khi stop() chuyển axis vào pha giảm tốc dừng.
   */
  bool isStopping() const;

  /**
   * @brief Đọc chế độ vận hành hiện tại.
   * @return TungLamStepperMode hiện tại.
   */
  TungLamStepperMode mode() const;

  /**
   * @brief Đọc tọa độ pulse hiện tại.
   * @return Position hiện tại [pulse], cập nhật sau mỗi STEP hoàn tất.
   */
  int32_t position() const;

  /**
   * @brief Đọc target pulse hiện tại.
   * @return Target position [pulse].
   */
  int32_t targetPosition() const;

  /**
   * @brief Gán lại tọa độ hiện tại mà không phát STEP.
   * @param positionSteps Position mới [pulse].
   * @note Chỉ dùng khi axis đang idle, thường sau calibration/homing ngoài.
   */
  void setCurrentPosition(int32_t positionSteps);

  /**
   * @brief Đọc khoảng cách còn lại tới target.
   * @return targetPosition - currentPosition [pulse], saturate ở biên int32.
   */
  int32_t distanceToGo() const;

  /**
   * @brief Đọc position theo số vòng output.
   * @return Position hiện tại [vòng output].
   */
  float positionRevolutions() const;

  /**
   * @brief Đọc position theo góc output.
   * @return Position hiện tại [độ].
   */
  float positionDegrees() const;

  /**
   * @brief Đọc position theo hành trình tuyến tính.
   * @return Position theo đơn vị travel đã cấu hình.
   */
  float positionTravel() const;

  /**
   * @brief Gắn công tắc MIN/MAX.
   * @param minPin Chân MIN/HOME hoặc NO_PIN.
   * @param maxPin Chân MAX hoặc NO_PIN.
   * @param activeLow true nếu switch tác động mức LOW.
   * @param usePullups true để dùng INPUT_PULLUP nội.
   * @return true nếu các chân yêu cầu hợp lệ và được attach thành công.
   */
  bool attachLimits(uint8_t minPin,
                    uint8_t maxPin,
                    bool activeLow = true,
                    bool usePullups = true);
  /** @brief Gỡ cả MIN/MAX limit khỏi axis. */
  void detachLimits();

  /**
   * @brief Đọc trạng thái công tắc MIN/HOME.
   * @return true nếu MIN đang tác động theo polarity đã cấu hình.
   */
  bool minLimitActive() const;

  /**
   * @brief Đọc trạng thái công tắc MAX.
   * @return true nếu MAX đang tác động theo polarity đã cấu hình.
   */
  bool maxLimitActive() const;

  /**
   * @brief Bật giới hạn mềm theo tọa độ pulse tuyệt đối.
   * @param minPosition Biên nhỏ nhất [pulse].
   * @param maxPosition Biên lớn nhất [pulse].
   * @note Nếu truyền ngược thứ tự, library tự hoán đổi MIN/MAX.
   */
  void setSoftLimits(int32_t minPosition, int32_t maxPosition);

  /** @brief Tắt giới hạn mềm; hard limit vật lý vẫn hoạt động nếu đã attach. */
  void clearSoftLimits();

  /**
   * @brief Kiểm tra bảo vệ giới hạn mềm.
   * @return true nếu soft-limit đang được bật.
   */
  bool softLimitsEnabled() const;

  /**
   * @brief Đọc fault gần nhất của axis.
   * @return TungLamStepperFault hiện tại.
   */
  TungLamStepperFault fault() const;

  /** @brief Xóa fault khi axis đang dừng. */
  void clearFault();

  /** @brief Đổi fault enum thành chuỗi PROGMEM để debug mà không tốn SRAM. */
  static const __FlashStringHelper* faultName(TungLamStepperFault fault);

  /** @brief Đổi mode enum thành chuỗi PROGMEM để debug. */
  static const __FlashStringHelper* modeName(TungLamStepperMode mode);

  /** @brief Đổi trạng thái homing enum thành chuỗi PROGMEM để debug. */
  static const __FlashStringHelper* homingStateName(
      TungLamStepperHomingState state);
  /**
   * @brief In snapshot trạng thái gọn ra Serial/Print.
   * @param out Đích Print, ví dụ Serial, Serial1 hoặc object Print tương thích.
   * @example axis.printState(Serial);
   */
  void printState(Print& out) const;

  /**
   * @brief Đọc chân STEP/PUL đã truyền vào constructor.
   * @return Số chân Arduino dùng cho STEP/PUL.
   */
  uint8_t stepPin() const;

  /**
   * @brief Đọc chân DIR đã truyền vào constructor.
   * @return Số chân Arduino dùng cho DIR.
   */
  uint8_t dirPin() const;

  /**
   * @brief Đọc chân ENA/ENABLE.
   * @return Số chân ENA/ENABLE hoặc NO_PIN nếu axis không dùng enable.
   */
  uint8_t enablePin() const;

  /**
   * @brief Đọc slot scheduler mà axis đang chiếm.
   * @return Slot 0..maxAxes()-1 hoặc NO_PIN nếu chưa begin().
   */
  uint8_t engineSlot() const;

  /**
   * @brief Trần tần số STEP theo pulse-width + timer guard hiện tại.
   * @return Tần số STEP cực đại lý thuyết [pulse/s].
   * @note Đây là trần timing lý thuyết; tải CPU thực tế giảm khi nhiều axis chạy đồng thời.
   */
  uint32_t maximumStepRate() const;

  /**
   * @brief Đọc số axis tối đa của scheduler build hiện tại.
   * @return Số slot axis tối đa.
   */
  static uint8_t maxAxes();

  /**
   * @brief Đọc tần số Timer1 nội bộ sau prescaler.
   * @return Tần số engine [Hz].
   */
  static uint32_t timerFrequencyHz();

 private:
  friend class tunglam::stepper::internal::TLTimerEngine;

  // Cached fast GPIO. Mỗi object giữ sẵn port register + bit mask để ISR không
  // phải gọi digitalWrite()/digitalRead() hoặc tra pin map lại ở runtime.
  tunglam::stepper::internal::TLFastPin step_;      ///< STEP/PUL output.
  tunglam::stepper::internal::TLFastPin dir_;       ///< DIR output.
  tunglam::stepper::internal::TLFastPin enable_;    ///< ENA output tùy chọn.
  tunglam::stepper::internal::TLFastPin limitMin_;  ///< MIN/HOME input.
  tunglam::stepper::internal::TLFastPin limitMax_;  ///< MAX input.
  tunglam::stepper::internal::TLFastPin ms1_;       ///< MS1/MODE0 output.
  tunglam::stepper::internal::TLFastPin ms2_;       ///< MS2/MODE1 output.
  tunglam::stepper::internal::TLFastPin ms3_;       ///< MS3/MODE2 output.

  const uint8_t stepPinNumber_;    ///< Arduino pin number của STEP.
  const uint8_t dirPinNumber_;     ///< Arduino pin number của DIR.
  const uint8_t enablePinNumber_;  ///< Arduino pin number của ENA hoặc NO_PIN.

  TungLamStepperDriverConfig driverConfig_;  ///< Timing/polarity driver hiện tại.
  TungLamStepperMotionConfig motionConfig_;  ///< Giới hạn planner hiện tại.
  uint16_t pulseWidthTicks_;                  ///< Pulse width đã cache theo Timer1 tick.
  uint16_t directionSetupTicks_;              ///< DIR setup đã cache theo Timer1 tick.

  volatile bool begun_;    ///< begin() đã thành công và axis đã có scheduler slot.
  volatile bool enabled_;  ///< Trạng thái ENA logic library đang quản lý.
  bool autoEnable_;        ///< Tự enable trước motion khi true.
  uint8_t slot_;           ///< Slot 0..kMaxAxes-1 trong shared scheduler.

  volatile TungLamStepperMode mode_;    ///< Idle/Position/Continuous/Homing.
  volatile TungLamStepperFault fault_;  ///< Fault gần nhất.
  volatile bool running_;               ///< Có motion event đang hoạt động.
  volatile bool stopping_;              ///< Đang controlled-stop sau stop().
  volatile bool pulseHigh_;             ///< STEP đang ở mức active, chờ Compare-B hạ.
  volatile uint16_t pulseFallAt_;        ///< TCNT1 tuyệt đối để kết thúc pulse.
  volatile uint32_t ticksToStep_;        ///< Tick còn lại tới STEP rising event tiếp theo.

  volatile int8_t directionSign_;        ///< +1/-1 theo chiều tọa độ đang chạy.
  volatile int32_t currentPosition_;     ///< Position hiện tại [pulse].
  volatile int32_t targetPosition_;      ///< Target position [pulse].

  // Trapezoid planner state. Các biến này được ISR cập nhật sau từng STEP.
  volatile uint32_t totalSteps_;         ///< Tổng pulse của move position hiện tại.
  volatile uint32_t completedSteps_;     ///< Pulse đã hoàn tất trong move.
  volatile uint32_t accelSteps_;         ///< Số pulse pha tăng tốc.
  volatile uint32_t cruiseSteps_;        ///< Số pulse pha tốc độ đều.
  volatile uint32_t decelSteps_;         ///< Số pulse pha giảm tốc.
  volatile int32_t accelN_;              ///< Chỉ số recurrence Q24.8 khi tăng tốc.
  volatile int32_t decelN_;              ///< Chỉ số recurrence khi giảm tốc.
  volatile uint32_t currentIntervalTicks_; ///< Chu kỳ STEP hiện tại [timer tick].
  volatile uint32_t currentIntervalQ8_;    ///< Chu kỳ STEP Q24.8 để giảm lượng tử.
  volatile uint32_t minIntervalTicks_;     ///< Chu kỳ nhỏ nhất theo maxSpeed.
  volatile uint32_t minIntervalQ8_;        ///< Chu kỳ nhỏ nhất ở dạng Q24.8.
  volatile uint32_t c0Ticks_;              ///< Chu kỳ STEP đầu tiên theo acceleration.
  volatile uint32_t c0Q8_;                 ///< c0 dạng Q24.8.

  // Homing state machine: seek nhanh -> nhả switch/backoff -> bắt lại chậm.
  volatile TungLamStepperHomingState homingState_; ///< Pha homing hiện tại.
  volatile bool homed_;                  ///< Đã homing thành công hay chưa.
  int8_t homingDirectionSign_;           ///< Hướng logic đi tới HOME.
  uint32_t homingFastIntervalTicks_;     ///< Chu kỳ seek nhanh.
  uint32_t homingSlowIntervalTicks_;     ///< Chu kỳ seek chậm.
  volatile uint32_t homingBackoffRemaining_; ///< Pulse backoff còn lại.
  bool homingReleasedLimit_;             ///< Đã xác nhận switch nhả trong Backoff.
  int32_t homingHomePosition_;           ///< Position gán khi homing hoàn tất.
  uint8_t homingConfirmSamples_;         ///< Số mẫu liên tiếp cần xác nhận.
  uint8_t homingActiveSamples_;          ///< Counter debounce trạng thái active.
  uint8_t homingInactiveSamples_;        ///< Counter debounce trạng thái released.
  uint32_t homingMaxPhaseSteps_;         ///< Travel guard mỗi pha; 0 = tắt.
  volatile uint32_t homingPhaseSteps_;   ///< Pulse đã đi trong pha homing hiện tại.

  bool limitsAttached_;   ///< Có ít nhất một hard-limit pin đang attach.
  bool limitsActiveLow_;  ///< true nếu switch active LOW.

  bool softLimitsEnabled_;  ///< Bật/tắt soft-limit.
  int32_t softMin_;         ///< Biên software MIN [pulse].
  int32_t softMax_;         ///< Biên software MAX [pulse].

  uint16_t fullStepsPerRevolution_;  ///< Full-step/rev của rotor.
  uint16_t microsteps_;              ///< Hệ số vi bước.
  float gearRatio_;                  ///< Motor rev / output rev.
  float travelPerOutputRevolution_;  ///< Hành trình / output rev theo đơn vị user.

  /** @brief Áp DIR logic mới và đảm bảo setup-time trước STEP đầu. */
  bool prepareDirection(int8_t sign);

  /** @brief Áp truth-table MS pins cho A4988/DRV8825. */
  bool applyMicrostepProfile(uint16_t microsteps,
                             TungLamMicrostepDriverProfile profile);

  /** @brief Lập trapezoid/triangle profile từ currentPosition tới target. */
  bool planPositionMove(int32_t target);

  /** @brief Kiểm tra profile motion có an toàn với recurrence int32 hay không. */
  bool validateMotionConfig(const TungLamStepperMotionConfig& config) const;

  /** @brief Kiểm tra pulse width/DIR setup/polarity của driver. */
  bool validateDriverConfig(const TungLamStepperDriverConfig& config) const;

  /** @brief Quy đổi timing microsecond sang tick một lần ngoài ISR và cache lại. */
  void rebuildDriverTimingCache();

  /** @brief Đổi tốc độ [pulse/s] thành chu kỳ timer tick bằng ceil division. */
  uint32_t intervalTicksForSpeed(uint32_t stepsPerSecond) const;

  /** @brief Tính c0 của recurrence tăng tốc từ acceleration. */
  uint32_t initialIntervalTicks(uint32_t acceleration) const;

  /** @brief Đổi microsecond thành Timer1 tick, luôn làm tròn lên. */
  uint32_t microsecondsToTimerTicks(uint16_t microseconds) const;

  /** @brief Chu kỳ STEP nhỏ nhất đảm bảo pulse width + compare guard. */
  uint32_t minimumLegalIntervalTicks() const;

  /** @brief Tính trần STEP rate theo timing của một driver config cụ thể. */
  uint32_t maximumStepRateForDriver(
      const TungLamStepperDriverConfig& config) const;

  /** @brief Đổi timer tick integer sang Q24.8. */
  static uint32_t ticksToQ8(uint32_t ticks);

  /** @brief Đổi Q24.8 về timer tick integer. */
  static uint32_t q8ToTicks(uint32_t q8);

  /** @brief ISR-safe: kiểm tra hard-limit theo hướng đang chạy. */
  bool directionLimitActiveFromIsr() const;

  /** @brief ISR-safe: đọc đúng switch HOME tương ứng hướng homing. */
  bool homeLimitActiveFromIsr() const;

  /** @brief ISR-safe: kiểm tra STEP tiếp theo có vi phạm soft-limit. */
  bool nextStepViolatesSoftLimitFromIsr() const;

  /** @brief ISR-safe: chặn currentPosition_ tràn int32 ở STEP tiếp theo. */
  bool nextStepOverflowsPositionFromIsr() const;

  /** @brief ISR-safe: xử lý debounce/chuyển pha homing trước khi phát STEP. */
  bool handleHomingBeforeStepFromIsr();

  /** @brief ISR-safe: kiểm tra travel guard của pha homing. */
  bool homingPhaseLimitExceededFromIsr() const;

  /** @brief ISR-safe: reset counter pulse của pha homing. */
  void resetHomingPhaseCounterFromIsr();

  /** @brief ISR-safe: chuyển SeekFast sang Backoff. */
  void transitionHomingToBackoffFromIsr();

  /** @brief ISR-safe: chuyển Backoff sang SeekSlow. */
  void transitionHomingToSlowSeekFromIsr();

  /** @brief ISR-safe: hoàn tất homing, set zero và homed flag. */
  void finishHomingFromIsr();

  /** @brief ISR-safe: tạo cạnh active của STEP và lên lịch pulse fall. */
  void stepRiseFromIsr();

  /** @brief ISR-safe: trả STEP về idle level. */
  void stepFallFromIsr();

  /** @brief ISR-safe: cập nhật position/planner sau một STEP hoàn tất. */
  void onStepCompletedFromIsr();

  /** @brief ISR-safe: dừng ngay và ghi fault. */
  void hardStopFromIsr(TungLamStepperFault fault);

  /** @brief ISR-safe: hoàn tất position move bình thường. */
  void finishMoveFromIsr();

  /** @brief Tính target controlled-stop từ tốc độ tức thời và giới hạn an toàn. */
  void recalculateDeceleratedStop();

  /** @brief Đọc limit pin và áp activeLow polarity. */
  bool readLimit(const tunglam::stepper::internal::TLFastPin& pin) const;

  /** @brief Làm tròn float sang int32 có kiểm tra finite/range. */
  static bool roundedFloatToInt32(float value, int32_t* out);
};
