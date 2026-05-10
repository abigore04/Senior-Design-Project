#### `metal_detector_avr_register/metal_detector_avr_register.ino`

This file is a register-level AVR rewrite of the original `metal_detector_fixed/metal_detector_fixed.ino` firmware. It performs the same metal detector reading task, but instead of using the `FreqCount` Arduino library and high-level Arduino functions, it directly configures the ATmega328P hardware registers.

The conditioned oscillator output from the metal detector circuit is connected to `D5 / PD5 / T1`, where Timer1 is used as an external pulse counter. Timer2 is configured to generate the 100 ms timing window for frequency measurement, and USART0 is used directly for serial communication with the Jetson Nano.

This version substitutes the original library-based implementation and provides lower-level control over the microcontroller. Direct register manipulation reduces software overhead, removes the dependency on the `FreqCount` library, and makes the firmware more transparent for timing-critical frequency counting. The output format remains the same as in the original code, so it can still be used with the existing ROS serial mapping node.

Output format: `difference,hit`

Example output:

    0,0
    2,0
    7,1

Hardware input:

    Detector output → Arduino Nano D5 / PD5 / T1
