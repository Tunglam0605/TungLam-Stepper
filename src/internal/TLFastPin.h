#pragma once

#include <Arduino.h>

namespace tunglam {
namespace stepper {
namespace internal {

/**
 * @brief Small cached-register GPIO wrapper for AVR hot paths.
 *
 * attach() resolves the Arduino pin only once, then high()/low()/read() operate
 * directly on the cached port register and bit mask. This is used inside Timer1
 * ISR paths so STEP/DIR/limit access does not pay digitalWrite()/digitalRead()
 * lookup overhead on every event.
 */
class TLFastPin {
 public:
  static constexpr uint8_t kNoPin = 0xFF;  ///< Sentinel for an unused optional pin.

  /** @brief Construct an unattached/invalid pin wrapper. */
  TLFastPin()
      : pin_(kNoPin), mask_(0), out_(nullptr), in_(nullptr), valid_(false) {}

  /**
   * @brief Resolve and cache an Arduino pin.
   * @param pin Arduino digital pin number.
   * @param mode INPUT, INPUT_PULLUP or OUTPUT.
   * @return true if the pin maps to valid AVR input/output registers.
   */
  bool attach(uint8_t pin, uint8_t mode) {
    pin_ = pin;
    if (pin == kNoPin) {
      valid_ = false;
      mask_ = 0;
      out_ = nullptr;
      in_ = nullptr;
      return false;
    }

    const uint8_t port = digitalPinToPort(pin);
    if (port == NOT_A_PIN) {
      valid_ = false;
      return false;
    }

    pinMode(pin, mode);
    mask_ = digitalPinToBitMask(pin);
    out_ = portOutputRegister(port);
    in_ = portInputRegister(port);
    valid_ = (mask_ != 0 && out_ != nullptr && in_ != nullptr);
    return valid_;
  }

  /** @return true if attach() resolved valid AVR registers. */
  inline bool valid() const { return valid_; }

  /** @return Arduino pin number or kNoPin when unused. */
  inline uint8_t pin() const { return pin_; }

  /** @brief Drive cached output bit HIGH. */
  inline void high() {
    if (valid_) *out_ |= mask_;
  }

  /** @brief Drive cached output bit LOW. */
  inline void low() {
    if (valid_) *out_ &= static_cast<uint8_t>(~mask_);
  }

  /**
   * @brief Drive an arbitrary logic level through cached register access.
   * @param highLevel true for HIGH, false for LOW.
   */
  inline void write(bool highLevel) {
    if (highLevel) {
      high();
    } else {
      low();
    }
  }

  /** @return Current cached input bit level; false when pin is invalid. */
  inline bool read() const {
    return valid_ && ((*in_ & mask_) != 0);
  }

 private:
  uint8_t pin_;            ///< Arduino pin number.
  uint8_t mask_;           ///< Bit mask inside AVR port register.
  volatile uint8_t* out_;  ///< Cached output register address.
  volatile uint8_t* in_;   ///< Cached input register address.
  bool valid_;             ///< attach() succeeded.
};

}  // namespace internal
}  // namespace stepper
}  // namespace tunglam
