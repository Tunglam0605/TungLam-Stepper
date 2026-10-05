#pragma once

#include <Arduino.h>

#ifndef TUNGLAM_STEPPER_MAX_AXES
#define TUNGLAM_STEPPER_MAX_AXES 4
#endif

class TungLamStepper;

namespace tunglam {
namespace stepper {
namespace internal {

class TLTimerEngine {
 public:
  static constexpr uint8_t kMaxAxes = TUNGLAM_STEPPER_MAX_AXES;
  static constexpr uint16_t kMaxScheduleChunkTicks = 60000;
  static constexpr uint16_t kMinCompareGuardTicks = 8;

  static bool registerAxis(TungLamStepper* axis, uint8_t* slotOut);
  static void unregisterAxis(TungLamStepper* axis);
  static void synchronizeNow();
  static void notifyScheduleChanged();
  static bool claimed();
  static uint32_t timerHz();

  static void onCompareA();
  static void onCompareB();

 private:
  static TungLamStepper* axes_[kMaxAxes];
  static volatile bool claimed_;
  static volatile uint16_t scheduledDeltaTicks_;
  static volatile uint16_t scheduledStartCounter_;

  static uint8_t savedTccr1a_;
  static uint8_t savedTccr1b_;
  static uint8_t savedTimsk1_;
  static uint16_t savedTcnt1_;
  static uint16_t savedOcr1a_;
  static uint16_t savedOcr1b_;

  static bool claimTimer1();
  static void releaseTimer1IfUnused();
  static void processRiseElapsedFromIsr(uint16_t elapsed);
  static void armNextRiseFromIsr();
  static void armNextFallFromIsr();
};

}  // namespace internal
}  // namespace stepper
}  // namespace tunglam
