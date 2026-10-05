#pragma once

#include <Arduino.h>

#ifndef TUNGLAM_STEPPER_MAX_AXES
#define TUNGLAM_STEPPER_MAX_AXES 4
#endif

class TungLamStepper;

namespace tunglam {
namespace stepper {
namespace internal {

/**
 * @brief Shared Timer1 event scheduler for every TungLamStepper axis.
 *
 * Compare-A schedules the next STEP rising edge among all active axes.
 * Compare-B schedules STEP falling edges so pulse width is independent from
 * the motion interval. The scheduler stores remaining ticks per axis instead
 * of dedicating one hardware timer/channel to each motor.
 *
 * Resource contract:
 * - owns AVR Timer1 only while at least one axis is registered;
 * - fails closed if Timer1 was already reconfigured by another subsystem;
 * - saves and restores the previous Timer1 register state when the final axis
 *   unregisters.
 */
class TLTimerEngine {
 public:
  static constexpr uint8_t kMaxAxes = TUNGLAM_STEPPER_MAX_AXES;  ///< Scheduler slots.
  static constexpr uint16_t kMaxScheduleChunkTicks = 60000;     ///< Long-wait chunk limit.
  static constexpr uint16_t kMinCompareGuardTicks = 8;           ///< Minimum safe compare lead.

  /**
   * @brief Register an axis with the shared scheduler.
   * @param axis Axis object to register.
   * @param slotOut Receives scheduler slot index.
   * @return true if Timer1 ownership and a free slot are available.
   */
  static bool registerAxis(TungLamStepper* axis, uint8_t* slotOut);

  /**
   * @brief Remove an axis and restore Timer1 if it was the final registered axis.
   * @param axis Axis to remove.
   */
  static void unregisterAxis(TungLamStepper* axis);

  /**
   * @brief Account for elapsed Timer1 time before application code changes a schedule.
   *
   * This prevents multi-axis timing drift when an axis starts/stops between compare
   * events: elapsed ticks are subtracted from every other active axis first.
   */
  static void synchronizeNow();

  /** @brief Recompute Compare-A/Compare-B after axis timing state changes. */
  static void notifyScheduleChanged();

  /** @return true while this library currently owns Timer1. */
  static bool claimed();

  /** @return Timer1 scheduler frequency [Hz] using prescaler /8. */
  static uint32_t timerHz();

  /** @brief Timer1 Compare-A ISR body: process next STEP rising event(s). */
  static void onCompareA();

  /** @brief Timer1 Compare-B ISR body: process due STEP falling edge(s). */
  static void onCompareB();

 private:
  static TungLamStepper* axes_[kMaxAxes];             ///< Registered axis pointers.
  static volatile bool claimed_;                      ///< Timer1 ownership flag.
  static volatile uint16_t scheduledDeltaTicks_;      ///< Current Compare-A interval.
  static volatile uint16_t scheduledStartCounter_;    ///< TCNT1 at interval start.

  // Snapshot restored after the final axis unregisters.
  static uint8_t savedTccr1a_;   ///< Original TCCR1A.
  static uint8_t savedTccr1b_;   ///< Original TCCR1B.
  static uint8_t savedTimsk1_;   ///< Original TIMSK1.
  static uint16_t savedTcnt1_;   ///< Original TCNT1.
  static uint16_t savedOcr1a_;   ///< Original OCR1A.
  static uint16_t savedOcr1b_;   ///< Original OCR1B.

  /** @brief Claim/configure Timer1 in normal mode, prescaler /8. */
  static bool claimTimer1();

  /** @brief Restore saved Timer1 registers when no Stepper axis remains. */
  static void releaseTimer1IfUnused();

  /**
   * @brief Subtract one elapsed scheduler slice and emit all due STEP rises.
   * @param elapsed Elapsed Timer1 ticks.
   */
  static void processRiseElapsedFromIsr(uint16_t elapsed);

  /** @brief Arm Compare-A for the nearest active axis STEP event. */
  static void armNextRiseFromIsr();

  /** @brief Arm Compare-B for the nearest pending STEP falling edge. */
  static void armNextFallFromIsr();
};

}  // namespace internal
}  // namespace stepper
}  // namespace tunglam
