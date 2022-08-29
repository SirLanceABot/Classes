**Take Back Half Controller (TBH)**

Control a system to a setpoint

The controller output is the time integral of the error of the setpoint compared to measured times the gain like the I in PID until you overshoot the setpoint (from either direction).

When the system overshoots the setpoint, use two output values to compute a corrected output:
- the output just before the overshoot that caused the overshoot (this is from memory)
- the (tentative) output after the overshoot that is calculated to return the process back toward the setpoint

The actual output of the controller is then calculated as the average of those two outputs - the previous one and the current tentative one.

The controller is tuned with a single gain parameter that relates the integrated process error to controller output. An accurate feedforward value for the desired setpoint helps the speed of convergence but isn't required.
