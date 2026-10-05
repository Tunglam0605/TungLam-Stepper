#include "TLTimerEngine.h"

#include "../TungLam_Stepper.h"

#if defined(__AVR__)
#include <avr/interrupt.h>
#include <avr/io.h>
#endif

namespace tunglam {
namespace stepper {
namespace internal {

TungLamStepper* TLTimerEngine::axes_[TLTimerEngine::kMaxAxes] = {};
volatile bool TLTimerEngine::claimed_ = false;
volatile uint16_t TLTimerEngine::scheduledDeltaTicks_ = 0;
volatile uint16_t TLTimerEngine::scheduledStartCounter_ = 0;

namespace {

static inline uint32_t tlMinU32(uint32_t a, uint32_t b) {
  return a < b ? a : b;
}

#if defined(__AVR__)
class AtomicGuard {
 public:
  AtomicGuard() : sreg_(SREG) { cli(); }
  ~AtomicGuard() { SREG = sreg_; }

 private:
  uint8_t sreg_;
};
#else
class AtomicGuard {
 public:
  AtomicGuard() { noInterrupts(); }
  ~AtomicGuard() { interrupts(); }
};
#endif

}  // namespace

bool TLTimerEngine::registerAxis(TungLamStepper* axis, uint8_t* slotOut) {
  if (axis == nullptr || slotOut == nullptr) return false;

  AtomicGuard lock;

  if (!claimed_ && !claimTimer1()) {
    return false;
  }

  for (uint8_t i = 0; i < kMaxAxes; ++i) {
    if (axes_[i] == axis) {
      *slotOut = i;
      return true;
    }
  }

  for (uint8_t i = 0; i < kMaxAxes; ++i) {
    if (axes_[i] == nullptr) {
      axes_[i] = axis;
      *slotOut = i;
      return true;
    }
  }

  return false;
}

void TLTimerEngine::unregisterAxis(TungLamStepper* axis) {
  if (axis == nullptr) return;

  synchronizeNow();
  AtomicGuard lock;
  for (uint8_t i = 0; i < kMaxAxes; ++i) {
    if (axes_[i] == axis) axes_[i] = nullptr;
  }
  armNextRiseFromIsr();
  armNextFallFromIsr();
}

void TLTimerEngine::synchronizeNow() {
#if defined(TIMSK1) && defined(TCNT1) && defined(TIFR1)
  AtomicGuard lock;

  if ((TIMSK1 & _BV(OCIE1A)) == 0 || scheduledDeltaTicks_ == 0) return;

  const uint16_t now = TCNT1;
  const uint16_t elapsed =
      static_cast<uint16_t>(now - scheduledStartCounter_);
  const bool comparePending = (TIFR1 & _BV(OCF1A)) != 0;

  if (comparePending || elapsed >= scheduledDeltaTicks_) {
    TIFR1 = _BV(OCF1A);
    const uint16_t scheduled = scheduledDeltaTicks_;
    scheduledDeltaTicks_ = 0;
    processRiseElapsedFromIsr(scheduled);
    armNextFallFromIsr();
    armNextRiseFromIsr();
    return;
  }

  if (elapsed == 0) return;

  for (uint8_t i = 0; i < kMaxAxes; ++i) {
    TungLamStepper* axis = axes_[i];
    if (axis == nullptr || !axis->running_) continue;

    if (axis->ticksToStep_ > elapsed) {
      axis->ticksToStep_ -= elapsed;
    } else {
      axis->ticksToStep_ = 0;
    }
  }

  scheduledDeltaTicks_ =
      static_cast<uint16_t>(scheduledDeltaTicks_ - elapsed);
  scheduledStartCounter_ = now;
  OCR1A = static_cast<uint16_t>(now + scheduledDeltaTicks_);
  TIFR1 = _BV(OCF1A);
#endif
}

void TLTimerEngine::notifyScheduleChanged() {
  AtomicGuard lock;
  armNextRiseFromIsr();
  armNextFallFromIsr();
}

bool TLTimerEngine::claimed() {
  return claimed_;
}

uint32_t TLTimerEngine::timerHz() {
#if defined(F_CPU)
  return static_cast<uint32_t>(F_CPU) / 8UL;
#else
  return 2000000UL;
#endif
}

bool TLTimerEngine::claimTimer1() {
#if !defined(TCCR1A) || !defined(TCCR1B) || !defined(TIMSK1)
  return false;
#else
  // Arduino AVR core configures Timer1 for analogWrite() even when no sketch
  // actively owns it. Treat enabled Timer1 interrupts as the real ownership
  // conflict, then deliberately reclaim the timer from the default PWM setup.
  const uint8_t busyInterrupts =
      _BV(OCIE1A) | _BV(OCIE1B) | _BV(TOIE1) | _BV(ICIE1);
  if ((TIMSK1 & busyInterrupts) != 0) return false;

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  OCR1A = 0;
  OCR1B = 0;

  // Normal mode, prescaler /8. At 16 MHz this gives a 2 MHz timer (0.5 us/tick).
  TCCR1B = _BV(CS11);
  TIMSK1 &= static_cast<uint8_t>(~(_BV(OCIE1A) | _BV(OCIE1B)));

  claimed_ = true;
  return true;
#endif
}

void TLTimerEngine::processRiseElapsedFromIsr(uint16_t elapsed) {
  for (uint8_t i = 0; i < kMaxAxes; ++i) {
    TungLamStepper* axis = axes_[i];
    if (axis == nullptr || !axis->running_) continue;

    if (axis->ticksToStep_ > elapsed) {
      axis->ticksToStep_ -= elapsed;
      continue;
    }

    axis->ticksToStep_ = 0;
    axis->stepRiseFromIsr();
  }
}

void TLTimerEngine::onCompareA() {
  const uint16_t elapsed = scheduledDeltaTicks_;
  scheduledDeltaTicks_ = 0;

  processRiseElapsedFromIsr(elapsed);
  armNextFallFromIsr();
  armNextRiseFromIsr();
}

void TLTimerEngine::onCompareB() {
  const uint16_t now = TCNT1;

  for (uint8_t i = 0; i < kMaxAxes; ++i) {
    TungLamStepper* axis = axes_[i];
    if (axis == nullptr || !axis->pulseHigh_) continue;

    if (static_cast<int16_t>(now - axis->pulseFallAt_) >= 0) {
      axis->stepFallFromIsr();
    }
  }

  armNextFallFromIsr();
}

void TLTimerEngine::armNextRiseFromIsr() {
#if defined(TIMSK1)
  uint32_t minimum = 0xFFFFFFFFUL;
  bool found = false;

  for (uint8_t i = 0; i < kMaxAxes; ++i) {
    TungLamStepper* axis = axes_[i];
    if (axis == nullptr || !axis->running_) continue;

    if (axis->ticksToStep_ < minimum) minimum = axis->ticksToStep_;
    found = true;
  }

  if (!found) {
    TIMSK1 &= static_cast<uint8_t>(~_BV(OCIE1A));
    scheduledDeltaTicks_ = 0;
    return;
  }

  uint16_t delta = static_cast<uint16_t>(
      tlMinU32(minimum, kMaxScheduleChunkTicks));

  if (delta < kMinCompareGuardTicks) delta = kMinCompareGuardTicks;

  scheduledDeltaTicks_ = delta;
  scheduledStartCounter_ = TCNT1;
  OCR1A = static_cast<uint16_t>(scheduledStartCounter_ + delta);
  TIFR1 = _BV(OCF1A);
  TIMSK1 |= _BV(OCIE1A);
#endif
}

void TLTimerEngine::armNextFallFromIsr() {
#if defined(TIMSK1)
  const uint16_t now = TCNT1;
  uint16_t minimumDelta = 0xFFFF;
  bool found = false;

  for (uint8_t i = 0; i < kMaxAxes; ++i) {
    TungLamStepper* axis = axes_[i];
    if (axis == nullptr || !axis->pulseHigh_) continue;

    uint16_t delta = static_cast<uint16_t>(axis->pulseFallAt_ - now);
    if (delta == 0 || delta > 0x7FFF) delta = 1;

    if (delta < minimumDelta) minimumDelta = delta;
    found = true;
  }

  if (!found) {
    TIMSK1 &= static_cast<uint8_t>(~_BV(OCIE1B));
    return;
  }

  OCR1B = static_cast<uint16_t>(now + minimumDelta);
  TIFR1 = _BV(OCF1B);
  TIMSK1 |= _BV(OCIE1B);
#endif
}

}  // namespace internal
}  // namespace stepper
}  // namespace tunglam

#if defined(TIMER1_COMPA_vect)
ISR(TIMER1_COMPA_vect) {
  tunglam::stepper::internal::TLTimerEngine::onCompareA();
}
#endif

#if defined(TIMER1_COMPB_vect)
ISR(TIMER1_COMPB_vect) {
  tunglam::stepper::internal::TLTimerEngine::onCompareB();
}
#endif
