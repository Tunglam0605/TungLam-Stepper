# Technical references

TungLam_Stepper is an original implementation. The following documents are used to verify electrical/timing behavior and motion-control design decisions.

## AVR / motion

- Microchip AVR446 / AN8017 — Linear speed control of stepper motor.
  https://www.microchip.com/en-us/application-notes/an8017
- ATmega640/1280/1281/2560/2561 datasheet — Timer/Counter1, GPIO and PORT mapping.
  https://www.microchip.com/en-us/product/ATmega2560

## Driver timing and microstep truth tables

- Allegro A4988 datasheet.
  https://www.allegromicro.com/en/products/motor-drivers/brush-dc-motor-drivers/a4988
- Texas Instruments DRV8825 datasheet.
  https://www.ti.com/product/DRV8825

The default STEP pulse width is intentionally conservative at 4 us. Driver-specific requirements can be configured through TungLamStepperDriverConfig.

## Architectural references

The public behavior was compared conceptually with mature Arduino stepper libraries such as AccelStepper, FastAccelStepper and MobaTools. TungLam_Stepper does not copy their source; its Timer1 scheduler, resource map and API are designed specifically around the TungLam Mega2560 robot stack.
