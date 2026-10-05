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

enum class TungLamStepperFault : uint8_t {
  None = 0,
  InvalidPin,
  NoAxisSlot,
  Timer1Conflict,
  InvalidConfig,
  SoftLimit,
  MinLimit,
  MaxLimit,
  PositionOverflow,
  HomingTravelExceeded
};

enum class TungLamStepperMode : uint8_t {
  Idle = 0,
  Position,
  Continuous,
  Homing
};

enum class TungLamStepperHomingState : uint8_t {
  Idle = 0,
  SeekFast,
  Backoff,
  SeekSlow,
  Complete
};

enum class TungLamStepperDirection : int8_t {
  Negative = -1,
  Positive = 1
};

enum class TungLamMicrostepDriverProfile : uint8_t {
  Manual = 0,
  A4988,
  DRV8825
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

  /** @return true nếu library đang coi driver ở trạng thái enable. */
  bool enabled() const;

  /**
   * @brief Bật/tắt tự enable driver khi bắt đầu một lệnh motion.
   * @note Mặc định true; kết thúc move không tự disable để giữ mô-men.
   */
  void setAutoEnable(bool enabled);
  bool autoEnable() const;

  bool setDriverConfig(const TungLamStepperDriverConfig& config);
  const TungLamStepperDriverConfig& driverConfig() const;

  bool setMotionConfig(const TungLamStepperMotionConfig& config);
  const TungLamStepperMotionConfig& motionConfig() const;
  bool setMaxSpeed(uint32_t stepsPerSecond);
  bool setAcceleration(uint32_t stepsPerSecond2);
  bool setDeceleration(uint32_t stepsPerSecond2);

  /** @brief Khai báo số full-step của motor cho một vòng rotor, ví dụ 200 với motor 1.8°. */
  bool setMotorFullStepsPerRevolution(uint16_t fullSteps);

  /**
   * @brief Khai báo hệ số vi bước đang được driver sử dụng.
   * @note Dùng trực tiếp cho TB6600/DM542 đặt microstep bằng DIP.
   */
  bool setMicrosteps(uint16_t microsteps);

  /**
   * @brief Gắn ba chân MS/MODE tùy chọn cho A4988/DRV8825.
   * @note Không bắt buộc; core motion vẫn hoạt động với driver đặt microstep bằng DIP.
   */
  bool attachMicrostepPins(uint8_t ms1Pin, uint8_t ms2Pin, uint8_t ms3Pin);
  void detachMicrostepPins();
  bool microstepPinsAttached() const;

  /** @brief Ghi trực tiếp ba mức logic MS1/MS2/MS3 theo nhu cầu custom. */
  bool setMicrostepPinLevels(bool ms1, bool ms2, bool ms3);

  /**
   * @brief Chọn vi bước bằng truth-table tích hợp của A4988 hoặc DRV8825.
   * @return false nếu profile/mức vi bước không hợp lệ hoặc chưa attach MS pins.
   */
  bool setMicrostepMode(uint16_t microsteps,
                        TungLamMicrostepDriverProfile profile);

  /**
   * @brief Đặt tỷ số truyền motor/output.
   * @param motorRevolutionsPerOutputRevolution Số vòng motor cho một vòng đầu ra.
   */
  bool setGearRatio(float motorRevolutionsPerOutputRevolution);

  /**
   * @brief Đặt hành trình tuyến tính của một vòng đầu ra.
   * @param travelUnits Có thể là mm/rev, cm/rev... miễn toàn project dùng cùng đơn vị.
   */
  bool setTravelPerOutputRevolution(float travelUnits);
  uint16_t motorFullStepsPerRevolution() const;
  uint16_t microsteps() const;
  float gearRatio() const;
  float travelPerOutputRevolution() const;
  float pulsesPerOutputRevolution() const;

  /** @brief Di chuyển tương đối số pulse/step với profile tăng/giảm tốc. */
  bool move(int32_t relativeSteps);

  /** @brief Di chuyển tới vị trí pulse tuyệt đối. */
  bool moveTo(int32_t absolutePositionSteps);

  /** @brief Di chuyển tương đối theo số vòng của trục đầu ra. */
  bool moveRevolutions(float outputRevolutions);

  /** @brief Di chuyển tương đối theo góc đầu ra [độ]. */
  bool moveDegrees(float outputDegrees);

  /** @brief Di chuyển tương đối theo đơn vị hành trình đã khai báo. */
  bool moveTravel(float travelUnits);

  /** @brief Đi tới vị trí tuyệt đối tính theo vòng đầu ra. */
  bool moveToRevolutions(float outputRevolutions);

  /** @brief Đi tới vị trí tuyệt đối tính theo độ đầu ra. */
  bool moveToDegrees(float outputDegrees);

  /** @brief Đi tới vị trí tuyệt đối theo đơn vị hành trình đã khai báo. */
  bool moveToTravel(float travelUnits);

  int32_t revolutionsToSteps(float outputRevolutions) const;
  int32_t degreesToSteps(float outputDegrees) const;
  int32_t travelToSteps(float travelUnits) const;
  float stepsToRevolutions(int32_t steps) const;
  float stepsToDegrees(int32_t steps) const;
  float stepsToTravel(int32_t steps) const;

  /**
   * @brief Chạy liên tục ở tốc độ pulse cố định.
   * @param direction Hướng Positive/Negative.
   * @param stepsPerSecond Tần số STEP [pulse/s], không được vượt maxSpeed đã cấu hình.
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

  /** @return true khi state machine homing đang hoạt động. */
  bool isHoming() const;

  /** @return true sau khi homing hoàn tất thành công. */
  bool isHomed() const;

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

  bool isRunning() const;
  bool isMovingToPosition() const;

  /** @return true khi stop() đã chuyển axis vào pha giảm tốc dừng. */
  bool isStopping() const;

  TungLamStepperMode mode() const;

  int32_t position() const;
  int32_t targetPosition() const;
  void setCurrentPosition(int32_t positionSteps);
  int32_t distanceToGo() const;

  float positionRevolutions() const;
  float positionDegrees() const;
  float positionTravel() const;

  /**
   * @brief Gắn công tắc MIN/MAX.
   * @param minPin Chân MIN/HOME hoặc NO_PIN.
   * @param maxPin Chân MAX hoặc NO_PIN.
   * @param activeLow true nếu switch tác động mức LOW.
   * @param usePullups true để dùng INPUT_PULLUP nội.
   */
  bool attachLimits(uint8_t minPin,
                    uint8_t maxPin,
                    bool activeLow = true,
                    bool usePullups = true);
  void detachLimits();
  bool minLimitActive() const;
  bool maxLimitActive() const;

  void setSoftLimits(int32_t minPosition, int32_t maxPosition);
  void clearSoftLimits();
  bool softLimitsEnabled() const;

  /** @return Fault gần nhất của axis. */
  TungLamStepperFault fault() const;

  /** @brief Xóa fault khi axis đang dừng. */
  void clearFault();

  /** @brief Đổi fault enum thành chuỗi PROGMEM để debug mà không tốn SRAM. */
  static const __FlashStringHelper* faultName(TungLamStepperFault fault);
  static const __FlashStringHelper* modeName(TungLamStepperMode mode);
  static const __FlashStringHelper* homingStateName(
      TungLamStepperHomingState state);
  /**
   * @brief In snapshot trạng thái gọn ra Serial/Print.
   * @example axis.printState(Serial);
   */
  void printState(Print& out) const;

  uint8_t stepPin() const;
  uint8_t dirPin() const;
  uint8_t enablePin() const;
  uint8_t engineSlot() const;

  /**
   * @brief Trần tần số STEP theo pulse-width + timer guard hiện tại.
   * @note Đây là trần timing lý thuyết; tải CPU thực tế giảm khi nhiều axis chạy đồng thời.
   */
  uint32_t maximumStepRate() const;

  static uint8_t maxAxes();
  static uint32_t timerFrequencyHz();

 private:
  friend class tunglam::stepper::internal::TLTimerEngine;

  tunglam::stepper::internal::TLFastPin step_;
  tunglam::stepper::internal::TLFastPin dir_;
  tunglam::stepper::internal::TLFastPin enable_;
  tunglam::stepper::internal::TLFastPin limitMin_;
  tunglam::stepper::internal::TLFastPin limitMax_;
  tunglam::stepper::internal::TLFastPin ms1_;
  tunglam::stepper::internal::TLFastPin ms2_;
  tunglam::stepper::internal::TLFastPin ms3_;

  const uint8_t stepPinNumber_;
  const uint8_t dirPinNumber_;
  const uint8_t enablePinNumber_;

  TungLamStepperDriverConfig driverConfig_;
  TungLamStepperMotionConfig motionConfig_;
  uint16_t pulseWidthTicks_;
  uint16_t directionSetupTicks_;

  volatile bool begun_;
  volatile bool enabled_;
  bool autoEnable_;
  uint8_t slot_;

  volatile TungLamStepperMode mode_;
  volatile TungLamStepperFault fault_;
  volatile bool running_;
  volatile bool stopping_;
  volatile bool pulseHigh_;
  volatile uint16_t pulseFallAt_;
  volatile uint32_t ticksToStep_;

  volatile int8_t directionSign_;
  volatile int32_t currentPosition_;
  volatile int32_t targetPosition_;

  volatile uint32_t totalSteps_;
  volatile uint32_t completedSteps_;
  volatile uint32_t accelSteps_;
  volatile uint32_t cruiseSteps_;
  volatile uint32_t decelSteps_;
  volatile int32_t accelN_;
  volatile int32_t decelN_;
  volatile uint32_t currentIntervalTicks_;
  volatile uint32_t currentIntervalQ8_;
  volatile uint32_t minIntervalTicks_;
  volatile uint32_t minIntervalQ8_;
  volatile uint32_t c0Ticks_;
  volatile uint32_t c0Q8_;

  volatile TungLamStepperHomingState homingState_;
  volatile bool homed_;
  int8_t homingDirectionSign_;
  uint32_t homingFastIntervalTicks_;
  uint32_t homingSlowIntervalTicks_;
  volatile uint32_t homingBackoffRemaining_;
  bool homingReleasedLimit_;
  int32_t homingHomePosition_;
  uint8_t homingConfirmSamples_;
  uint8_t homingActiveSamples_;
  uint8_t homingInactiveSamples_;
  uint32_t homingMaxPhaseSteps_;
  volatile uint32_t homingPhaseSteps_;

  bool limitsAttached_;
  bool limitsActiveLow_;

  bool softLimitsEnabled_;
  int32_t softMin_;
  int32_t softMax_;

  uint16_t fullStepsPerRevolution_;
  uint16_t microsteps_;
  float gearRatio_;
  float travelPerOutputRevolution_;

  bool prepareDirection(int8_t sign);
  bool applyMicrostepProfile(uint16_t microsteps,
                             TungLamMicrostepDriverProfile profile);
  bool planPositionMove(int32_t target);
  bool validateMotionConfig(const TungLamStepperMotionConfig& config) const;
  bool validateDriverConfig(const TungLamStepperDriverConfig& config) const;
  void rebuildDriverTimingCache();

  uint32_t intervalTicksForSpeed(uint32_t stepsPerSecond) const;
  uint32_t initialIntervalTicks(uint32_t acceleration) const;
  uint32_t microsecondsToTimerTicks(uint16_t microseconds) const;
  uint32_t minimumLegalIntervalTicks() const;
  uint32_t maximumStepRateForDriver(
      const TungLamStepperDriverConfig& config) const;
  static uint32_t ticksToQ8(uint32_t ticks);
  static uint32_t q8ToTicks(uint32_t q8);

  bool directionLimitActiveFromIsr() const;
  bool homeLimitActiveFromIsr() const;
  bool nextStepViolatesSoftLimitFromIsr() const;
  bool nextStepOverflowsPositionFromIsr() const;
  bool handleHomingBeforeStepFromIsr();
  bool homingPhaseLimitExceededFromIsr() const;
  void resetHomingPhaseCounterFromIsr();
  void transitionHomingToBackoffFromIsr();
  void transitionHomingToSlowSeekFromIsr();
  void finishHomingFromIsr();
  void stepRiseFromIsr();
  void stepFallFromIsr();
  void onStepCompletedFromIsr();
  void hardStopFromIsr(TungLamStepperFault fault);
  void finishMoveFromIsr();
  void recalculateDeceleratedStop();

  bool readLimit(const tunglam::stepper::internal::TLFastPin& pin) const;
  static bool roundedFloatToInt32(float value, int32_t* out);
};
