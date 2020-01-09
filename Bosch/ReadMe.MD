This code interprets the Bosch Seat Motor integrated Hall effect encoder to determine rotational position of the motor shaft.

The output of the encoder is a digital-type signal but the voltage drop and span from high to low is not within the range
that the roboRIO can interpret as a digital signal.

The encoder voltage is within the roboRIO's analog input range so the code in this project uses the analog input to "decode"
the signal.

If a digital input to the roboRIO is desired, the voltage level must be changed.  An AndyMark kit am-3812 can be procured and
assembled.  Instructions on the mechanical use of the motor and the electrical use of the motor are on AndyMark and ScreenStepsLive.

https://www.andymark.com/products/bosch-seat-motor-dio-kit

https://andymark-weblinc.netdna-ssl.com/media/W1siZiIsIjIwMTkvMDEvMjUvMTUvNDYvNTIvMmYwZjgzN2MtMTA4OC00OTMzLTk2YzEtODY3NzQwZDgzNmJjL1NlYXQgdG8gRElPIEFzc2VtYmx5IFBpY3R1cmVzIChhbS0zODEyKS5wZGYiXV0/Seat%20to%20DIO%20Assembly%20Pictures%20%28am-3812%29.pdf?sha=ce067c918f7d09a2

https://www.andymark.com/products/bosch-seat-motor-and-harness-cable

http://wpilib.screenstepslive.com/s/4485/m/63630/l/679357-bosch-seat-motor
