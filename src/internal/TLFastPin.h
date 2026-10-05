#pragma once

#include <Arduino.h>

namespace tunglam {
namespace stepper {
namespace internal {

class TLFastPin {
 public:
  static constexpr uint8_t kNoPin = 0xFF;

  TLFastPin()
      : pin_(kNoPin), mask_(0), out_(nullptr), in_(nullptr), valid_(false) {}

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

  inline bool valid() const { return valid_; }
  inline uint8_t pin() const { return pin_; }

  inline void high() {
    if (valid_) *out_ |= mask_;
  }

  inline void low() {
    if (valid_) *out_ &= static_cast<uint8_t>(~mask_);
  }

  inline void write(bool highLevel) {
    if (highLevel) {
      high();
    } else {
      low();
    }
  }

  inline bool read() const {
    return valid_ && ((*in_ & mask_) != 0);
  }

 private:
  uint8_t pin_;
  uint8_t mask_;
  volatile uint8_t* out_;
  volatile uint8_t* in_;
  bool valid_;
};

}  // namespace internal
}  // namespace stepper
}  // namespace tunglam
