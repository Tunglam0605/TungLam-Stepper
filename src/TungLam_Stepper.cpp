#include "TungLam_Stepper.h"

#include <math.h>

#include "internal/TLTimerEngine.h"

#if defined(__AVR__)
#include <avr/interrupt.h>
#include <avr/io.h>
#endif

using tunglam::stepper::internal::TLTimerEngine;

namespace {

static inline uint32_t tlMinU32(uint32_t a, uint32_t b) {
  return a < b ? a : b;
}

static inline uint32_t tlMaxU32(uint32_t a, uint32_t b) {
  return a > b ? a : b;
}

class AtomicGuard {
 public:
  AtomicGuard() {
#if defined(__AVR__)
    sreg_ = SREG;
    cli();
#else
    noInterrupts();
#endif
  }

  ~AtomicGuard() {
#if defined(__AVR__)
    SREG = sreg_;
#else
    interrupts();
#endif
  }

 private:
#if defined(__AVR__)
  uint8_t sreg_;
#endif
};

}  // namespace

TungLamStepper::TungLamStepper(uint8_t stepPin,
                               uint8_t dirPin,
                               uint8_t enablePin)
    : stepPinNumber_(stepPin),
      dirPinNumber_(dirPin),
      enablePinNumber_(enablePin),
      begun_(false),
      enabled_(false),
      autoEnable_(true),
      slot_(NO_PIN),
      mode_(TungLamStepperMode::Idle),
      fault_(TungLamStepperFault::None),
      running_(false),
      pulseHigh_(false),
      pulseFallAt_(0),
      ticksToStep_(0),
      directionSign_(1),
      currentPosition_(0),
      targetPosition_(0),
      totalSteps_(0),
      completedSteps_(0),
      accelSteps_(0),
      cruiseSteps_(0),
      decelSteps_(0),
      accelN_(0),
      decelN_(0),
      currentIntervalTicks_(0),
      currentIntervalQ8_(0),
      minIntervalTicks_(0),
      minIntervalQ8_(0),
      c0Ticks_(0),
      c0Q8_(0),
      homingState_(TungLamStepperHomingState::Idle),
      homed_(false),
      homingDirectionSign_(-1),
      homingFastIntervalTicks_(0),
      homingSlowIntervalTicks_(0),
      homingBackoffRemaining_(0),
      homingReleasedLimit_(false),
      homingHomePosition_(0),
      homingConfirmSamples_(3),
      homingActiveSamples_(0),
      homingInactiveSamples_(0),
      limitsAttached_(false),
      limitsActiveLow_(true),
      softLimitsEnabled_(false),
      softMin_(0),
      softMax_(0),
      fullStepsPerRevolution_(200),
      microsteps_(1),
      gearRatio_(1.0f),
      travelPerOutputRevolution_(0.0f) {}

TungLamStepper::~TungLamStepper() {
  emergencyStop();
  TLTimerEngine::unregisterAxis(this);
}

bool TungLamStepper::begin() {
  return begin(driverConfig_, motionConfig_);
}

bool TungLamStepper::begin(const TungLamStepperDriverConfig& driver,
                           const TungLamStepperMotionConfig& motion) {
  if (begun_) return true;

  if (!validateDriverConfig(driver) || !validateMotionConfig(motion)) {
    fault_ = TungLamStepperFault::InvalidConfig;
    return false;
  }

  if (!step_.attach(stepPinNumber_, OUTPUT) ||
      !dir_.attach(dirPinNumber_, OUTPUT)) {
    fault_ = TungLamStepperFault::InvalidPin;
    return false;
  }

  if (enablePinNumber_ != NO_PIN && !enable_.attach(enablePinNumber_, OUTPUT)) {
    fault_ = TungLamStepperFault::InvalidPin;
    return false;
  }

  driverConfig_ = driver;
  motionConfig_ = motion;

  step_.low();
  dir_.low();

  if (enable_.valid()) {
    enable_.write(driverConfig_.enableActiveLow);
    enabled_ = false;
  } else {
    enabled_ = true;
  }

  uint8_t newSlot = NO_PIN;
  if (!TLTimerEngine::registerAxis(this, &newSlot)) {
    fault_ = TLTimerEngine::claimed()
                 ? TungLamStepperFault::NoAxisSlot
                 : TungLamStepperFault::Timer1Conflict;
    return false;
  }

  slot_ = newSlot;
  begun_ = true;
  fault_ = TungLamStepperFault::None;
  return true;
}

void TungLamStepper::enable() {
  if (enable_.valid()) {
    enable_.write(!driverConfig_.enableActiveLow);
  }
  enabled_ = true;
}

void TungLamStepper::disable() {
  emergencyStop();
  if (enable_.valid()) {
    enable_.write(driverConfig_.enableActiveLow);
  }
  enabled_ = false;
}

bool TungLamStepper::enabled() const {
  return enabled_;
}

void TungLamStepper::setAutoEnable(bool enabled) {
  autoEnable_ = enabled;
}

bool TungLamStepper::autoEnable() const {
  return autoEnable_;
}

bool TungLamStepper::setDriverConfig(
    const TungLamStepperDriverConfig& config) {
  if (!validateDriverConfig(config) || running_) return false;

  driverConfig_ = config;

  if (enable_.valid()) {
    enable_.write(enabled_ ? !driverConfig_.enableActiveLow
                           : driverConfig_.enableActiveLow);
  }
  return true;
}

const TungLamStepperDriverConfig& TungLamStepper::driverConfig() const {
  return driverConfig_;
}

bool TungLamStepper::setMotionConfig(
    const TungLamStepperMotionConfig& config) {
  if (!validateMotionConfig(config) || running_) return false;
  motionConfig_ = config;
  return true;
}

const TungLamStepperMotionConfig& TungLamStepper::motionConfig() const {
  return motionConfig_;
}

bool TungLamStepper::setMaxSpeed(uint32_t stepsPerSecond) {
  TungLamStepperMotionConfig config = motionConfig_;
  config.maxSpeedStepsPerSecond = stepsPerSecond;
  return setMotionConfig(config);
}

bool TungLamStepper::setAcceleration(uint32_t stepsPerSecond2) {
  TungLamStepperMotionConfig config = motionConfig_;
  config.accelerationStepsPerSecond2 = stepsPerSecond2;
  return setMotionConfig(config);
}

bool TungLamStepper::setDeceleration(uint32_t stepsPerSecond2) {
  TungLamStepperMotionConfig config = motionConfig_;
  config.decelerationStepsPerSecond2 = stepsPerSecond2;
  return setMotionConfig(config);
}

bool TungLamStepper::setMotorFullStepsPerRevolution(uint16_t fullSteps) {
  if (fullSteps == 0) return false;
  fullStepsPerRevolution_ = fullSteps;
  return true;
}

bool TungLamStepper::setMicrosteps(uint16_t microsteps) {
  if (microsteps == 0 || microsteps > 1024 || running_) return false;
  microsteps_ = microsteps;
  return true;
}

bool TungLamStepper::attachMicrostepPins(uint8_t ms1Pin,
                                         uint8_t ms2Pin,
                                         uint8_t ms3Pin) {
  if (running_) return false;

  tunglam::stepper::internal::TLFastPin p1;
  tunglam::stepper::internal::TLFastPin p2;
  tunglam::stepper::internal::TLFastPin p3;

  if (!p1.attach(ms1Pin, OUTPUT) ||
      !p2.attach(ms2Pin, OUTPUT) ||
      !p3.attach(ms3Pin, OUTPUT)) {
    fault_ = TungLamStepperFault::InvalidPin;
    return false;
  }

  ms1_ = p1;
  ms2_ = p2;
  ms3_ = p3;
  return true;
}

void TungLamStepper::detachMicrostepPins() {
  if (running_) return;
  ms1_ = tunglam::stepper::internal::TLFastPin();
  ms2_ = tunglam::stepper::internal::TLFastPin();
  ms3_ = tunglam::stepper::internal::TLFastPin();
}

bool TungLamStepper::microstepPinsAttached() const {
  return ms1_.valid() && ms2_.valid() && ms3_.valid();
}

bool TungLamStepper::setMicrostepPinLevels(bool ms1,
                                           bool ms2,
                                           bool ms3) {
  if (running_ || !microstepPinsAttached()) return false;
  ms1_.write(ms1);
  ms2_.write(ms2);
  ms3_.write(ms3);
  return true;
}

bool TungLamStepper::setMicrostepMode(
    uint16_t microsteps,
    TungLamMicrostepDriverProfile profile) {
  if (running_) return false;

  if (profile == TungLamMicrostepDriverProfile::Manual) {
    return setMicrosteps(microsteps);
  }

  if (!microstepPinsAttached()) {
    fault_ = TungLamStepperFault::InvalidPin;
    return false;
  }

  if (!applyMicrostepProfile(microsteps, profile)) {
    fault_ = TungLamStepperFault::InvalidConfig;
    return false;
  }

  microsteps_ = microsteps;
  fault_ = TungLamStepperFault::None;
  return true;
}

bool TungLamStepper::setGearRatio(float motorRevolutionsPerOutputRevolution) {
  if (!(motorRevolutionsPerOutputRevolution > 0.0f)) return false;
  gearRatio_ = motorRevolutionsPerOutputRevolution;
  return true;
}

bool TungLamStepper::setTravelPerOutputRevolution(float travelUnits) {
  if (!(travelUnits > 0.0f)) return false;
  travelPerOutputRevolution_ = travelUnits;
  return true;
}

uint16_t TungLamStepper::motorFullStepsPerRevolution() const {
  return fullStepsPerRevolution_;
}

uint16_t TungLamStepper::microsteps() const {
  return microsteps_;
}

float TungLamStepper::gearRatio() const {
  return gearRatio_;
}

float TungLamStepper::travelPerOutputRevolution() const {
  return travelPerOutputRevolution_;
}

float TungLamStepper::pulsesPerOutputRevolution() const {
  return static_cast<float>(fullStepsPerRevolution_) *
         static_cast<float>(microsteps_) * gearRatio_;
}

bool TungLamStepper::move(int32_t relativeSteps) {
  if (relativeSteps == 0) return true;

  const int64_t target =
      static_cast<int64_t>(position()) + static_cast<int64_t>(relativeSteps);
  if (target < -2147483648LL || target > 2147483647LL) {
    fault_ = TungLamStepperFault::InvalidConfig;
    return false;
  }

  return moveTo(static_cast<int32_t>(target));
}

bool TungLamStepper::moveTo(int32_t absolutePositionSteps) {
  if (!begun_) return false;

  if (softLimitsEnabled_ &&
      (absolutePositionSteps < softMin_ || absolutePositionSteps > softMax_)) {
    fault_ = TungLamStepperFault::SoftLimit;
    return false;
  }

  return planPositionMove(absolutePositionSteps);
}

bool TungLamStepper::moveRevolutions(float outputRevolutions) {
  const float pulses = outputRevolutions * pulsesPerOutputRevolution();
  if (!isfinite(pulses) || pulses < -2147483648.0f ||
      pulses > 2147483520.0f) {
    fault_ = TungLamStepperFault::InvalidConfig;
    return false;
  }
  return move(static_cast<int32_t>(lroundf(pulses)));
}

bool TungLamStepper::moveDegrees(float outputDegrees) {
  return moveRevolutions(outputDegrees / 360.0f);
}

bool TungLamStepper::moveTravel(float travelUnits) {
  if (!(travelPerOutputRevolution_ > 0.0f)) {
    fault_ = TungLamStepperFault::InvalidConfig;
    return false;
  }

  const float revolutions = travelUnits / travelPerOutputRevolution_;
  return moveRevolutions(revolutions);
}

bool TungLamStepper::runContinuous(TungLamStepperDirection direction,
                                   uint32_t stepsPerSecond) {
  if (!begun_ || stepsPerSecond == 0 || running_) return false;

  TLTimerEngine::synchronizeNow();

  const int8_t sign = static_cast<int8_t>(direction);
  if (!prepareDirection(sign)) return false;

  const uint32_t interval = intervalTicksForSpeed(stepsPerSecond);
  if (interval < minimumLegalIntervalTicks()) {
    fault_ = TungLamStepperFault::InvalidConfig;
    return false;
  }

  {
    AtomicGuard lock;
    mode_ = TungLamStepperMode::Continuous;
    running_ = true;
    directionSign_ = sign;
    totalSteps_ = 0;
    completedSteps_ = 0;
    currentIntervalTicks_ = interval;
    currentIntervalQ8_ = ticksToQ8(interval);
    minIntervalTicks_ = interval;
    minIntervalQ8_ = currentIntervalQ8_;
    ticksToStep_ = tlMaxU32(
        interval,
        static_cast<uint32_t>(driverConfig_.directionSetupUs) *
            (TLTimerEngine::timerHz() / 1000000UL));
  }

  if (autoEnable_) enable();
  fault_ = TungLamStepperFault::None;
  TLTimerEngine::notifyScheduleChanged();
  return true;
}

bool TungLamStepper::home(const TungLamStepperHomingConfig& config) {
  if (!begun_ || running_ || !limitsAttached_) return false;

  const int8_t sign = static_cast<int8_t>(config.direction);
  if (sign != 1 && sign != -1) return false;

  if (config.fastSpeedStepsPerSecond == 0 ||
      config.slowSpeedStepsPerSecond == 0 ||
      config.slowSpeedStepsPerSecond > config.fastSpeedStepsPerSecond ||
      config.confirmSamples == 0) {
    fault_ = TungLamStepperFault::InvalidConfig;
    return false;
  }

  const bool homePinPresent =
      sign < 0 ? limitMin_.valid() : limitMax_.valid();
  if (!homePinPresent) {
    fault_ = TungLamStepperFault::InvalidPin;
    return false;
  }

  const uint32_t fastInterval =
      intervalTicksForSpeed(config.fastSpeedStepsPerSecond);
  const uint32_t slowInterval =
      intervalTicksForSpeed(config.slowSpeedStepsPerSecond);

  if (fastInterval < minimumLegalIntervalTicks() ||
      slowInterval < minimumLegalIntervalTicks()) {
    fault_ = TungLamStepperFault::InvalidConfig;
    return false;
  }

  TLTimerEngine::synchronizeNow();

  homingDirectionSign_ = sign;
  homingFastIntervalTicks_ = fastInterval;
  homingSlowIntervalTicks_ = slowInterval;
  homingBackoffRemaining_ = config.backoffSteps;
  homingReleasedLimit_ = false;
  homingHomePosition_ = config.homePositionSteps;
  homingConfirmSamples_ = config.confirmSamples;
  homingActiveSamples_ = 0;
  homingInactiveSamples_ = 0;
  homed_ = false;
  fault_ = TungLamStepperFault::None;

  mode_ = TungLamStepperMode::Homing;
  running_ = true;
  completedSteps_ = 0;
  totalSteps_ = 0;

  if (homeLimitActiveFromIsr()) {
    homingState_ = TungLamStepperHomingState::Backoff;
    prepareDirection(-homingDirectionSign_);
    currentIntervalTicks_ = homingSlowIntervalTicks_;
  } else {
    homingState_ = TungLamStepperHomingState::SeekFast;
    prepareDirection(homingDirectionSign_);
    currentIntervalTicks_ = homingFastIntervalTicks_;
  }

  currentIntervalQ8_ = ticksToQ8(currentIntervalTicks_);
  minIntervalTicks_ = currentIntervalTicks_;
  minIntervalQ8_ = currentIntervalQ8_;

  const uint32_t setupTicks =
      static_cast<uint32_t>(driverConfig_.directionSetupUs) *
      tlMaxU32(1, TLTimerEngine::timerHz() / 1000000UL);
  ticksToStep_ = tlMaxU32(currentIntervalTicks_, setupTicks);

  if (autoEnable_) enable();
  TLTimerEngine::notifyScheduleChanged();
  return true;
}

bool TungLamStepper::isHoming() const {
  return running_ && mode_ == TungLamStepperMode::Homing;
}

bool TungLamStepper::isHomed() const {
  return homed_;
}

TungLamStepperHomingState TungLamStepper::homingState() const {
  return homingState_;
}

void TungLamStepper::clearHomed() {
  if (!running_) homed_ = false;
}

void TungLamStepper::stop() {
  if (!running_) return;

  if (mode_ == TungLamStepperMode::Homing) {
    emergencyStop();
    return;
  }

  TLTimerEngine::synchronizeNow();
  recalculateDeceleratedStop();
  TLTimerEngine::notifyScheduleChanged();
}

void TungLamStepper::emergencyStop() {
  TLTimerEngine::synchronizeNow();

  {
    AtomicGuard lock;
    const bool wasHoming = (mode_ == TungLamStepperMode::Homing);
    running_ = false;
    mode_ = TungLamStepperMode::Idle;
    ticksToStep_ = 0;

    if (wasHoming) {
      homingState_ = TungLamStepperHomingState::Idle;
      homed_ = false;
      homingActiveSamples_ = 0;
      homingInactiveSamples_ = 0;
    }
  }

  if (pulseHigh_) step_.low();
  pulseHigh_ = false;
  TLTimerEngine::notifyScheduleChanged();
}

bool TungLamStepper::isRunning() const {
  return running_;
}

bool TungLamStepper::isMovingToPosition() const {
  return running_ && mode_ == TungLamStepperMode::Position;
}

TungLamStepperMode TungLamStepper::mode() const {
  return mode_;
}

int32_t TungLamStepper::position() const {
  AtomicGuard lock;
  return currentPosition_;
}

int32_t TungLamStepper::targetPosition() const {
  AtomicGuard lock;
  return targetPosition_;
}

void TungLamStepper::setCurrentPosition(int32_t positionSteps) {
  AtomicGuard lock;
  if (running_) return;
  currentPosition_ = positionSteps;
  targetPosition_ = positionSteps;
}

int32_t TungLamStepper::distanceToGo() const {
  AtomicGuard lock;
  return targetPosition_ - currentPosition_;
}

float TungLamStepper::positionRevolutions() const {
  const float ppr = pulsesPerOutputRevolution();
  return ppr > 0.0f ? static_cast<float>(position()) / ppr : 0.0f;
}

float TungLamStepper::positionDegrees() const {
  return positionRevolutions() * 360.0f;
}

float TungLamStepper::positionTravel() const {
  if (!(travelPerOutputRevolution_ > 0.0f)) return 0.0f;
  return positionRevolutions() * travelPerOutputRevolution_;
}

bool TungLamStepper::attachLimits(uint8_t minPin,
                                  uint8_t maxPin,
                                  bool activeLow,
                                  bool usePullups) {
  if (running_) return false;

  const uint8_t mode = usePullups ? INPUT_PULLUP : INPUT;
  bool ok = true;

  if (minPin != NO_PIN) ok &= limitMin_.attach(minPin, mode);
  if (maxPin != NO_PIN) ok &= limitMax_.attach(maxPin, mode);

  if (!ok) {
    fault_ = TungLamStepperFault::InvalidPin;
    return false;
  }

  limitsAttached_ = (minPin != NO_PIN || maxPin != NO_PIN);
  limitsActiveLow_ = activeLow;
  return true;
}

void TungLamStepper::detachLimits() {
  if (running_) return;
  limitMin_ = tunglam::stepper::internal::TLFastPin();
  limitMax_ = tunglam::stepper::internal::TLFastPin();
  limitsAttached_ = false;
}

bool TungLamStepper::minLimitActive() const {
  return readLimit(limitMin_);
}

bool TungLamStepper::maxLimitActive() const {
  return readLimit(limitMax_);
}

void TungLamStepper::setSoftLimits(int32_t minPosition, int32_t maxPosition) {
  if (minPosition > maxPosition) {
    const int32_t temp = minPosition;
    minPosition = maxPosition;
    maxPosition = temp;
  }

  softMin_ = minPosition;
  softMax_ = maxPosition;
  softLimitsEnabled_ = true;
}

void TungLamStepper::clearSoftLimits() {
  softLimitsEnabled_ = false;
}

bool TungLamStepper::softLimitsEnabled() const {
  return softLimitsEnabled_;
}

TungLamStepperFault TungLamStepper::fault() const {
  return fault_;
}

void TungLamStepper::clearFault() {
  if (!running_) fault_ = TungLamStepperFault::None;
}

uint8_t TungLamStepper::stepPin() const { return stepPinNumber_; }
uint8_t TungLamStepper::dirPin() const { return dirPinNumber_; }
uint8_t TungLamStepper::enablePin() const { return enablePinNumber_; }
uint8_t TungLamStepper::engineSlot() const { return slot_; }

uint8_t TungLamStepper::maxAxes() {
  return TLTimerEngine::kMaxAxes;
}

uint32_t TungLamStepper::timerFrequencyHz() {
  return TLTimerEngine::timerHz();
}

bool TungLamStepper::prepareDirection(int8_t sign) {
  if (sign != 1 && sign != -1) return false;
  const bool positive = (sign > 0);
  dir_.write(positive ^ driverConfig_.directionInverted);
  directionSign_ = sign;
  return true;
}

bool TungLamStepper::applyMicrostepProfile(
    uint16_t microsteps,
    TungLamMicrostepDriverProfile profile) {
  bool b1 = false;
  bool b2 = false;
  bool b3 = false;

  if (profile == TungLamMicrostepDriverProfile::A4988) {
    switch (microsteps) {
      case 1:  b1 = false; b2 = false; b3 = false; break;
      case 2:  b1 = true;  b2 = false; b3 = false; break;
      case 4:  b1 = false; b2 = true;  b3 = false; break;
      case 8:  b1 = true;  b2 = true;  b3 = false; break;
      case 16: b1 = true;  b2 = true;  b3 = true;  break;
      default: return false;
    }
  } else if (profile == TungLamMicrostepDriverProfile::DRV8825) {
    // DRV8825 MODE0/MODE1/MODE2 are mapped to ms1/ms2/ms3.
    switch (microsteps) {
      case 1:  b1 = false; b2 = false; b3 = false; break;
      case 2:  b1 = true;  b2 = false; b3 = false; break;
      case 4:  b1 = false; b2 = true;  b3 = false; break;
      case 8:  b1 = true;  b2 = true;  b3 = false; break;
      case 16: b1 = false; b2 = false; b3 = true;  break;
      case 32: b1 = true;  b2 = false; b3 = true;  break;
      default: return false;
    }
  } else {
    return false;
  }

  return setMicrostepPinLevels(b1, b2, b3);
}

bool TungLamStepper::planPositionMove(int32_t target) {
  if (running_) return false;

  TLTimerEngine::synchronizeNow();
  const int32_t current = position();
  const int64_t delta64 =
      static_cast<int64_t>(target) - static_cast<int64_t>(current);
  if (delta64 == 0) return true;

  const uint32_t steps =
      static_cast<uint32_t>(delta64 > 0 ? delta64 : -delta64);
  const int8_t sign = delta64 > 0 ? 1 : -1;

  if (!prepareDirection(sign)) return false;

  const float vmax =
      static_cast<float>(motionConfig_.maxSpeedStepsPerSecond);
  const float accel =
      static_cast<float>(motionConfig_.accelerationStepsPerSecond2);
  const float decel =
      static_cast<float>(motionConfig_.decelerationStepsPerSecond2);

  float peak2 = vmax * vmax;
  uint32_t accelSteps =
      static_cast<uint32_t>(peak2 / (2.0f * accel));
  uint32_t decelSteps =
      static_cast<uint32_t>(peak2 / (2.0f * decel));

  float peakSpeed = vmax;

  if (static_cast<uint64_t>(accelSteps) + decelSteps > steps) {
    peak2 = (2.0f * static_cast<float>(steps) * accel * decel) /
            (accel + decel);
    peakSpeed = sqrtf(peak2);
    accelSteps =
        static_cast<uint32_t>(peak2 / (2.0f * accel));
    if (accelSteps >= steps) accelSteps = steps > 1 ? steps - 1 : 0;
    decelSteps = steps - accelSteps;
  }

  const uint32_t cruiseSteps =
      steps - tlMinU32(steps, accelSteps + decelSteps);

  uint32_t minInterval =
      intervalTicksForSpeed(tlMaxU32(1, static_cast<uint32_t>(peakSpeed)));
  minInterval = tlMaxU32(minInterval, minimumLegalIntervalTicks());

  uint32_t c0 = initialIntervalTicks(
      motionConfig_.accelerationStepsPerSecond2);
  c0 = tlMaxU32(c0, minInterval);

  {
    AtomicGuard lock;
    targetPosition_ = target;
    mode_ = TungLamStepperMode::Position;
    running_ = true;
    directionSign_ = sign;

    totalSteps_ = steps;
    completedSteps_ = 0;
    accelSteps_ = accelSteps;
    cruiseSteps_ = cruiseSteps;
    decelSteps_ = decelSteps;

    accelN_ = 0;
    decelN_ = -static_cast<int32_t>(tlMaxU32(1, decelSteps));

    minIntervalTicks_ = minInterval;
    minIntervalQ8_ = ticksToQ8(minInterval);
    c0Ticks_ = c0;
    c0Q8_ = ticksToQ8(c0);
    currentIntervalQ8_ = c0Q8_;
    currentIntervalTicks_ = q8ToTicks(currentIntervalQ8_);
    ticksToStep_ = currentIntervalTicks_;
  }

  if (autoEnable_) enable();
  fault_ = TungLamStepperFault::None;
  TLTimerEngine::notifyScheduleChanged();
  return true;
}

bool TungLamStepper::validateMotionConfig(
    const TungLamStepperMotionConfig& config) const {
  return config.maxSpeedStepsPerSecond > 0 &&
         config.accelerationStepsPerSecond2 > 0 &&
         config.decelerationStepsPerSecond2 > 0;
}

bool TungLamStepper::validateDriverConfig(
    const TungLamStepperDriverConfig& config) const {
  return config.pulseWidthUs >= 2 && config.pulseWidthUs <= 100 &&
         config.directionSetupUs <= 1000;
}

uint32_t TungLamStepper::intervalTicksForSpeed(
    uint32_t stepsPerSecond) const {
  if (stepsPerSecond == 0) return 0xFFFFFFFFUL;
  const uint32_t hz = TLTimerEngine::timerHz();
  return tlMaxU32(1, hz / stepsPerSecond);
}

uint32_t TungLamStepper::initialIntervalTicks(uint32_t acceleration) const {
  if (acceleration == 0) return 0xFFFFFFFFUL;

  const float timerHz = static_cast<float>(TLTimerEngine::timerHz());
  const float c0 = 0.676f * timerHz *
                   sqrtf(2.0f / static_cast<float>(acceleration));

  if (c0 < 1.0f) return 1;
  if (c0 > 4294960000.0f) return 0xFFFFFFFFUL;
  return static_cast<uint32_t>(c0);
}

uint32_t TungLamStepper::minimumLegalIntervalTicks() const {
  const uint32_t ticksPerUs =
      tlMaxU32(1, TLTimerEngine::timerHz() / 1000000UL);
  const uint32_t pulseTicks =
      static_cast<uint32_t>(driverConfig_.pulseWidthUs) * ticksPerUs;
  return pulseTicks + TLTimerEngine::kMinCompareGuardTicks;
}

uint32_t TungLamStepper::ticksToQ8(uint32_t ticks) {
  // Q24.8 keeps sub-tick precision for AVR446 recurrence while still
  // supporting intervals up to about 4.19 s at the 2 MHz Mega timer rate.
  const uint32_t kMaxIntegerTicks = 0x007FFFFFUL;
  if (ticks > kMaxIntegerTicks) ticks = kMaxIntegerTicks;
  return ticks << 8;
}

uint32_t TungLamStepper::q8ToTicks(uint32_t q8) {
  const uint32_t whole = q8 >> 8;
  return whole + ((q8 & 0xFFU) != 0 ? 1U : 0U);
}

bool TungLamStepper::directionLimitActiveFromIsr() const {
  if (!limitsAttached_) return false;
  if (directionSign_ < 0 && readLimit(limitMin_)) return true;
  if (directionSign_ > 0 && readLimit(limitMax_)) return true;
  return false;
}

bool TungLamStepper::homeLimitActiveFromIsr() const {
  if (homingDirectionSign_ < 0) return readLimit(limitMin_);
  return readLimit(limitMax_);
}

bool TungLamStepper::nextStepViolatesSoftLimitFromIsr() const {
  if (!softLimitsEnabled_) return false;
  const int32_t next = currentPosition_ + directionSign_;
  return next < softMin_ || next > softMax_;
}

void TungLamStepper::stepRiseFromIsr() {
  if (!running_) return;

  if (mode_ == TungLamStepperMode::Homing) {
    if (handleHomingBeforeStepFromIsr()) return;
  }

  if (directionLimitActiveFromIsr()) {
    hardStopFromIsr(directionSign_ < 0
                        ? TungLamStepperFault::MinLimit
                        : TungLamStepperFault::MaxLimit);
    return;
  }

  if (nextStepViolatesSoftLimitFromIsr()) {
    hardStopFromIsr(TungLamStepperFault::SoftLimit);
    return;
  }

  step_.high();
  pulseHigh_ = true;

  const uint32_t ticksPerUs =
      tlMaxU32(1, TLTimerEngine::timerHz() / 1000000UL);
  pulseFallAt_ = static_cast<uint16_t>(
      TCNT1 + static_cast<uint16_t>(
                  driverConfig_.pulseWidthUs * ticksPerUs));

  currentPosition_ += directionSign_;
  ++completedSteps_;

  if (mode_ == TungLamStepperMode::Homing &&
      homingState_ == TungLamStepperHomingState::Backoff) {
    if (!homeLimitActiveFromIsr()) {
      if (homingInactiveSamples_ < 255) ++homingInactiveSamples_;
    } else {
      homingInactiveSamples_ = 0;
    }

    if (!homingReleasedLimit_ &&
        homingInactiveSamples_ >= homingConfirmSamples_) {
      homingReleasedLimit_ = true;
      homingInactiveSamples_ = 0;
    } else if (homingReleasedLimit_ && homingBackoffRemaining_ > 0) {
      --homingBackoffRemaining_;
    }

    if (homingReleasedLimit_ && homingBackoffRemaining_ == 0) {
      transitionHomingToSlowSeekFromIsr();
      return;
    }
  }

  onStepCompletedFromIsr();
}

void TungLamStepper::stepFallFromIsr() {
  step_.low();
  pulseHigh_ = false;
}

bool TungLamStepper::handleHomingBeforeStepFromIsr() {
  if (homingState_ == TungLamStepperHomingState::SeekFast ||
      homingState_ == TungLamStepperHomingState::SeekSlow) {
    if (homeLimitActiveFromIsr()) {
      if (homingActiveSamples_ < 255) ++homingActiveSamples_;
    } else {
      homingActiveSamples_ = 0;
    }

    if (homingActiveSamples_ >= homingConfirmSamples_) {
      homingActiveSamples_ = 0;
      if (homingState_ == TungLamStepperHomingState::SeekFast) {
        transitionHomingToBackoffFromIsr();
      } else {
        finishHomingFromIsr();
      }
      return true;
    }
  }

  // While backing away, only the opposite-direction hard limit remains active.
  if (homingState_ == TungLamStepperHomingState::Backoff &&
      directionLimitActiveFromIsr()) {
    hardStopFromIsr(directionSign_ < 0
                        ? TungLamStepperFault::MinLimit
                        : TungLamStepperFault::MaxLimit);
    homingState_ = TungLamStepperHomingState::Idle;
    return true;
  }

  return false;
}

void TungLamStepper::transitionHomingToBackoffFromIsr() {
  homingState_ = TungLamStepperHomingState::Backoff;
  homingReleasedLimit_ = false;
  homingActiveSamples_ = 0;
  homingInactiveSamples_ = 0;
  prepareDirection(-homingDirectionSign_);
  currentIntervalTicks_ = homingSlowIntervalTicks_;
  currentIntervalQ8_ = ticksToQ8(currentIntervalTicks_);

  const uint32_t setupTicks =
      static_cast<uint32_t>(driverConfig_.directionSetupUs) *
      tlMaxU32(1, TLTimerEngine::timerHz() / 1000000UL);
  ticksToStep_ = tlMaxU32(currentIntervalTicks_, setupTicks);
}

void TungLamStepper::transitionHomingToSlowSeekFromIsr() {
  homingState_ = TungLamStepperHomingState::SeekSlow;
  homingActiveSamples_ = 0;
  homingInactiveSamples_ = 0;
  prepareDirection(homingDirectionSign_);
  currentIntervalTicks_ = homingSlowIntervalTicks_;
  currentIntervalQ8_ = ticksToQ8(currentIntervalTicks_);

  const uint32_t setupTicks =
      static_cast<uint32_t>(driverConfig_.directionSetupUs) *
      tlMaxU32(1, TLTimerEngine::timerHz() / 1000000UL);
  ticksToStep_ = tlMaxU32(currentIntervalTicks_, setupTicks);
}

void TungLamStepper::finishHomingFromIsr() {
  running_ = false;
  mode_ = TungLamStepperMode::Idle;
  homingState_ = TungLamStepperHomingState::Complete;
  homed_ = true;
  homingActiveSamples_ = 0;
  homingInactiveSamples_ = 0;
  ticksToStep_ = 0;
  currentPosition_ = homingHomePosition_;
  targetPosition_ = homingHomePosition_;
  fault_ = TungLamStepperFault::None;
}

void TungLamStepper::onStepCompletedFromIsr() {
  if (!running_) return;

  if (mode_ == TungLamStepperMode::Homing) {
    ticksToStep_ = currentIntervalTicks_;
    return;
  }

  if (mode_ == TungLamStepperMode::Continuous) {
    ticksToStep_ = currentIntervalTicks_;
    return;
  }

  if (completedSteps_ >= totalSteps_) {
    finishMoveFromIsr();
    return;
  }

  if (completedSteps_ < accelSteps_) {
    ++accelN_;
    const uint32_t denominator =
        static_cast<uint32_t>(4L * accelN_ + 1L);

    if (denominator > 0) {
      const uint32_t numerator = 2UL * currentIntervalQ8_;
      const uint32_t delta = numerator / denominator;
      if (delta < currentIntervalQ8_) currentIntervalQ8_ -= delta;
    }

    if (currentIntervalQ8_ < minIntervalQ8_) {
      currentIntervalQ8_ = minIntervalQ8_;
    }
  } else if (completedSteps_ < accelSteps_ + cruiseSteps_) {
    currentIntervalQ8_ = minIntervalQ8_;
  } else {
    if (decelN_ >= 0) {
      decelN_ = -static_cast<int32_t>(
          tlMaxU32(1, totalSteps_ - completedSteps_));
    }

    const int32_t denominatorSigned = 4L * decelN_ + 1L;
    if (denominatorSigned < 0) {
      const uint32_t denominator =
          static_cast<uint32_t>(-denominatorSigned);
      const uint32_t numerator = 2UL * currentIntervalQ8_;
      const uint32_t delta = numerator / denominator;

      const uint32_t kMaxQ8 = 0x7FFFFFFFUL;
      if (delta > kMaxQ8 - currentIntervalQ8_) {
        currentIntervalQ8_ = kMaxQ8;
      } else {
        currentIntervalQ8_ += delta;
      }
    }
    ++decelN_;
  }

  if (currentIntervalQ8_ < minIntervalQ8_) {
    currentIntervalQ8_ = minIntervalQ8_;
  }

  currentIntervalTicks_ = q8ToTicks(currentIntervalQ8_);
  currentIntervalTicks_ =
      tlMaxU32(currentIntervalTicks_, minimumLegalIntervalTicks());
  ticksToStep_ = currentIntervalTicks_;
}

void TungLamStepper::hardStopFromIsr(TungLamStepperFault fault) {
  running_ = false;
  mode_ = TungLamStepperMode::Idle;
  ticksToStep_ = 0;
  fault_ = fault;
  if (homingState_ != TungLamStepperHomingState::Complete) {
    homingState_ = TungLamStepperHomingState::Idle;
    homed_ = false;
  }
}

void TungLamStepper::finishMoveFromIsr() {
  running_ = false;
  mode_ = TungLamStepperMode::Idle;
  ticksToStep_ = 0;
  targetPosition_ = currentPosition_;
}

void TungLamStepper::recalculateDeceleratedStop() {
  AtomicGuard lock;

  if (!running_) return;

  const uint32_t interval = tlMaxU32(1, currentIntervalTicks_);
  const uint32_t speed = TLTimerEngine::timerHz() / interval;
  const uint32_t decel =
      tlMaxU32(1, motionConfig_.decelerationStepsPerSecond2);

  uint32_t stopSteps = static_cast<uint32_t>(
      (static_cast<uint64_t>(speed) * speed + (2UL * decel - 1)) /
      (2UL * decel));
  stopSteps = tlMaxU32(1, stopSteps);

  if (mode_ == TungLamStepperMode::Continuous) {
    mode_ = TungLamStepperMode::Position;
    totalSteps_ = stopSteps;
    completedSteps_ = 0;
    accelSteps_ = 0;
    cruiseSteps_ = 0;
  } else {
    totalSteps_ = completedSteps_ + stopSteps;
    accelSteps_ = completedSteps_;
    cruiseSteps_ = 0;
  }

  decelSteps_ = stopSteps;
  decelN_ = -static_cast<int32_t>(stopSteps);

  int64_t target =
      static_cast<int64_t>(currentPosition_) +
      static_cast<int64_t>(directionSign_) * stopSteps;

  if (softLimitsEnabled_) {
    if (target < softMin_) target = softMin_;
    if (target > softMax_) target = softMax_;
  }

  if (target < -2147483648LL) target = -2147483648LL;
  if (target > 2147483647LL) target = 2147483647LL;
  targetPosition_ = static_cast<int32_t>(target);
}

bool TungLamStepper::readLimit(
    const tunglam::stepper::internal::TLFastPin& pin) const {
  if (!pin.valid()) return false;
  const bool raw = pin.read();
  return limitsActiveLow_ ? !raw : raw;
}

uint32_t TungLamStepper::absSteps(int32_t delta) {
  if (delta >= 0) return static_cast<uint32_t>(delta);
  return static_cast<uint32_t>(-(static_cast<int64_t>(delta)));
}
