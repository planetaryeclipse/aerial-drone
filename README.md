# aerial-drone
This is an in-progress repo that stores my current firmware for an STM32-F401RE development board to be used in a drone that I am currently designing.

The firmware is very much in-progress and I will continue with this project when I get some more spare time. Currently I have communication going with my sensors (minus the GPS due to time constraints) and have been able to read from a controller on the side of the controller computer and can send a command over a serial radio to the microcontroller. This packet is then decoded and then changes the position of connected servos.
