# Technical references

TungLam_Stepper is an original implementation. These documents/repositories are used to verify timing, register ownership and motion-control decisions.

## AVR / motion

- Microchip AVR446 / AN8017 — Linear speed control of stepper motor.
  https://www.microchip.com/en-us/application-notes/an8017
- ATmega640/1280/1281/2560/2561 datasheet — Timer/Counter1, GPIO and PORT mapping.
  https://www.microchip.com/en-us/product/ATmega2560
- Arduino AVR Core wiring.c — default Timer1 configuration used by the ownership check.
  https://github.com/arduino/ArduinoCore-avr/blob/master/cores/arduino/wiring.c

## Driver timing and microstep truth tables

- Allegro A4988 datasheet.
  https://www.allegromicro.com/en/products/motor-drivers/brush-dc-motor-drivers/a4988
- Texas Instruments DRV8825 datasheet.
  https://www.ti.com/product/DRV8825

The default STEP pulse width is intentionally conservative at 4 us. Driver-specific timing is configurable with TungLamStepperDriverConfig.

A4988 and DRV8825 software microstep profiles follow their published MS/MODE truth tables.

## Architectural references

The design was compared conceptually with mature Arduino stepper libraries such as AccelStepper, FastAccelStepper and MobaTools.

TungLam_Stepper does not copy their source. Its Timer1 scheduler, resource ownership, pin baseline and API are designed specifically around the TungLam Mega2560 robotics stack.
