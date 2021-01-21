# Code Readme

The folder ProofOfConcept contains working programs for individual elements of Quest 1. 

countdown.c file displays a hard-coded hour-minute countdown on the 14-segment display, and prints the current second to console.

servo_wiggle.c sweeps the servo to the right and then sweeps left three times. 

These proofs of concepts are then used in the fish_feeder directory. Go into that directory, then into main, then fish_feeder.c to view the code of the fish_feeder we have designed. Currently, it is set to count down 0hr and 1min, but these macros can be adjusted for any hour-minute time. Alphanumeric display shows time in HHMM, but if the hour reaches zero, displays time in MMSS. Prints current display and second values to console. Uses GPIO 18 as PWM control.

