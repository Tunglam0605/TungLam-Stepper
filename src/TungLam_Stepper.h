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
  MaxLimit
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

struct TungLamStepperDriverConfig {
  bool enableActiveLow;
  bool directionInverted;
  uint16_t pulseWidthUs;
  uint16_t directionSetupUs;

  TungLamStepperDriverConfig(bool enActiveLow = true,
                             bool dirInverted = false,
                             uint16_t pulseUs = 4,
                             uint16_t dirSetup = 4)
      : enableActiveLow(enActiveLow),
        directionInverted(dirInverted),
        pulseWidthUs(pulseUs),
        directionSetupUs(dirSetup) {}
};

struct TungLamStepperHomingConfig {
  TungLamStepperDirection direction;
  uint32_t fastSpeedStepsPerSecond;
  uint32_t slowSpeedStepsPerSecond;
  uint32_t backoffSteps;
  int32_t homePositionSteps;
  uint8_t confirmSamples;

  TungLamStepperHomingConfig(
      TungLamStepperDirection homeDirection = TungLamStepperDirection::Negative,
      uint32_t fastSpeed = 2000,
      uint32_t slowSpeed = 400,
      uint32_t backoff = 100,
      int32_t homePosition = 0,
      uint8_t debounceSamples = 3)
      : direction(homeDirection),
        fastSpeedStepsPerSecond(fastSpeed),
        slowSpeedStepsPerSecond(slowSpeed),
        backoffSteps(backoff),
        homePositionSteps(homePosition),
        confirmSamples(debounceSamples) {}
};

struct TungLamStepperMotionConfig {
  uint32_t maxSpeedStepsPerSecond;
  uint32_t accelerationStepsPerSecond2;
  uint32_t decelerationStepsPerSecond2;

  TungLamStepperMotionConfig(uint32_t maxSpeed = 4000,
                             uint32_t acceleration = 8000,
                             uint32_t deceleration = 8000)
      : maxSpeedStepsPerSecond(maxSpeed),
        accelerationStepsPerSecond2(acceleration),
        decelerationStepsPerSecond2(deceleration) {}
};

class TungLamStepper {
 public:
  static constexpr uint8_t NO_PIN = 0xFF;

  TungLamStepper(uint8_t stepPin, uint8_t dirPin, uint8_t enablePin = NO_PIN);
  ~TungLamStepper();

  bool begin();
  bool begin(const TungLamStepperDriverConfig& driver,
             const TungLamStepperMotionConfig& motion);

  void enable();
  void disable();
  bool enabled() const;
  void setAutoEnable(bool enabled);
  bool autoEnable() const;

  bool setDriverConfig(const TungLamStepperDriverConfig& config);
  const TungLamStepperDriverConfig& driverConfig() const;

  bool setMotionConfig(const TungLamStepperMotionConfig& config);
  const TungLamStepperMotionConfig& motionConfig() const;
  bool setMaxSpeed(uint32_t stepsPerSecond);
  bool setAcceleration(uint32_t stepsPerSecond2);
  bool setDeceleration(uint32_t stepsPerSecond2);

  bool setMotorFullStepsPerRevolution(uint16_t fullSteps);
  bool setMicrosteps(uint16_t microsteps);
  bool attachMicrostepPins(uint8_t ms1Pin, uint8_t ms2Pin, uint8_t ms3Pin);
  void detachMicrostepPins();
  bool microstepPinsAttached() const;
  bool setMicrostepPinLevels(bool ms1, bool ms2, bool ms3);
  bool setMicrostepMode(uint16_t microsteps,
                        TungLamMicrostepDriverProfile profile);

  bool setGearRatio(float motorRevolutionsPerOutputRevolution);
  bool setTravelPerOutputRevolution(float travelUnits);
  uint16_t motorFullStepsPerRevolution() const;
  uint16_t microsteps() const;
  float gearRatio() const;
  float travelPerOutputRevolution() const;
  float pulsesPerOutputRevolution() const;

  bool move(int32_t relativeSteps);
  bool moveTo(int32_t absolutePositionSteps);
  bool moveRevolutions(float outputRevolutions);
  bool moveDegrees(float outputDegrees);
  bool moveTravel(float travelUnits);

  bool runContinuous(TungLamStepperDirection direction,
                     uint32_t stepsPerSecond);

  bool home(const TungLamStepperHomingConfig& config =
                TungLamStepperHomingConfig());
  bool isHoming() const;
  bool isHomed() const;
  TungLamStepperHomingState homingState() const;
  void clearHomed();

  void stop();
  void emergencyStop();

  bool isRunning() const;
  bool isMovingToPosition() const;
  TungLamStepperMode mode() const;

  int32_t position() const;
  int32_t targetPosition() const;
  void setCurrentPosition(int32_t positionSteps);
  int32_t distanceToGo() const;

  float positionRevolutions() const;
  float positionDegrees() const;
  float positionTravel() const;

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

  TungLamStepperFault fault() const;
  void clearFault();

  uint8_t stepPin() const;
  uint8_t dirPin() const;
  uint8_t enablePin() const;
  uint8_t engineSlot() const;

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

  volatile bool begun_;
  volatile bool enabled_;
  bool autoEnable_;
  uint8_t slot_;

  volatile TungLamStepperMode mode_;
  volatile TungLamStepperFault fault_;
  volatile bool running_;
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

  uint32_t intervalTicksForSpeed(uint32_t stepsPerSecond) const;
  uint32_t initialIntervalTicks(uint32_t acceleration) const;
  uint32_t minimumLegalIntervalTicks() const;
  static uint32_t ticksToQ8(uint32_t ticks);
  static uint32_t q8ToTicks(uint32_t q8);

  bool directionLimitActiveFromIsr() const;
  bool homeLimitActiveFromIsr() const;
  bool nextStepViolatesSoftLimitFromIsr() const;
  bool handleHomingBeforeStepFromIsr();
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
  static uint32_t absSteps(int32_t delta);
};
