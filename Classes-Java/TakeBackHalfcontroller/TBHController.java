package frc.robot;

// TBH Take Back Half Controller

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;

public class TBHController implements Sendable {
  private static int instances; // sequence number for multiple instances

  private static final double defaultTolerance = 0.05; // something to get you started
  private double m_tolerance;

  private static final double defaultFeedforward = .1; // something to get you started

  private double defaultSetpoint = 0.; // something to get you started
  private double m_setpoint;

  private double m_measurement = 0.;

  private static final double defaultGain = 1.e-5; // something to get you started
  private double m_gain;
  
  // before changing the setpoint from 0 to desired, each time set these initial values
  double m_prev_error;
  double m_output;
  double m_feedForward; // estimated speed (0 to 1) for the goal (setpoint)
  double m_tbh;

  /**
   * Creates a new Take-Back-Half controller.
   *
   * @param gain to correct the error
   * @param tolerance Tolerance for {@link #atSetpoint() atSetpoint}.
   * Tolerance is not used for this TBH controller and is provide as a convenience if the user
   * wants to use it for other additional control to know if atSetpoint.
   */
  public TBHController(double gain, double tolerance) {
    instances++; // sequence number for multiple instances

    setGain(gain);
    setTolerance(tolerance);
    setSetpoint(defaultSetpoint); // something to get you started

    SendableRegistry.addLW(this, "TBHController", instances);
  }

  /**
   * Creates a new Take-Back-Half controller.
   * Using default Gain and Tolerance
   *
   */
  public TBHController() {
    this(defaultGain, defaultTolerance); // something to get you started
  }

  /**
   * Sets the setpoint for the TBH controller.
   *
   * @param setpoint The desired setpoint (default setpoint is 0.0).
   * @param feedForward Approximate control signal needed for the setpoint
   * used to initiate ramp-up to speed.
   * It's range is [0, 1] with default 0.1 if omitted. 0 would give slower
   * ramp to speed with less overshoot. 1 would give fastest arrival at
   * speed and likely more overshoot.
   */
  public void setSetpoint(double setpoint, double feedForward) {
    m_setpoint = setpoint;
    m_feedForward = feedForward;
    // before changing the setpoint from 0 to desired, each time set these initial values
    m_prev_error = 1.; // indicates measurement was lower than setpoint - assuming new setpoint is higher than previous
    m_output = 1; // maximum positive control to raise next measurement as fast as possible until the setpoint is exceeded
    m_tbh = 2.*m_feedForward - 1.; // when run through calculate of TBH this results in the specified feedForward
  }

/**
   * Sets the setpoint for the TBH controller.
   *
   * @param setpoint The desired setpoint.
   * The default Feed Forward value is used.
   */
  public void setSetpoint(double setpoint) {
    m_setpoint = setpoint;
    setSetpoint(m_setpoint, defaultFeedforward);
  }

  /**
   * Returns the current setpoint of the TBH controller.
   *
   * @return The current setpoint.
   */
  public double getSetpoint() {
    return m_setpoint;
  }

  /**
   * Sets the gain for the TBH controller.
   *
   * @param gain The desired gain.
   */
  public void setGain(double gain) {
    m_gain = gain;
  }

  /**
   * Returns the current gain of the TBH controller.
   *
   * @return The current gain.
   */
  public double getGain() {
    return m_gain;
  }

  /**
   * Returns true if the error is within the tolerance of the setpoint.
   *
   * @return Whether the error is within the acceptable bounds.
   */
  public boolean atSetpoint() {
    return Math.abs(m_setpoint - m_measurement) < m_tolerance*m_setpoint;
  }

  /**
   * Sets the error within which atSetpoint will return true.
   *
   * @param tolerance Position error which is tolerable.
   */
  public void setTolerance(double tolerance) {
    m_tolerance = tolerance;
  }

  /**
   * Returns the current tolerance of the controller.
   *
   * @return The current tolerance.
   */
  public double getTolerance() {
    return m_tolerance;
  }

  /**
   * Returns the current measurement of the process variable.
   *
   * @return The current measurement of the process variable.
   */
  public double getMeasurement() {
    return m_measurement;
  }

  /**
   * Returns the current error.
   *
   * @return The current error.
   */
  public double getError() {
    return m_setpoint - m_measurement;
  }

  /**
   * Returns the calculated control output.
   *
   * @param measurement The most recent measurement of the process variable.
   * @return The calculated motor input (controller output).
   */
  public double calculate(double measurement) {
    if(m_setpoint == 0.) return 0.; // returning other than 0 for setpoint 0 is likely unsafe

    m_measurement = measurement;

    var error = m_setpoint - measurement;         // calculate the error;
    m_output += m_gain * error;                   // integrate the output;
  
    if (error >= 0 != m_prev_error >= 0)          // if zero crossing,
    { 
      m_output = 0.5 * (m_output + m_tbh);        // then Take Back Half
      m_tbh = m_output;                           // update Take Back Half variable
      m_prev_error = error;                       // and save the previous error
    }
      return edu.wpi.first.math.MathUtil.clamp(m_output, 0., 1.);
  }

  // for the LiveWindow stuff
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("TBHController");
    builder.addDoubleProperty("gain", this::getGain, this::setGain);
    builder.addDoubleProperty("tolerance", this::getTolerance, this::setTolerance);
    builder.addDoubleProperty("setpoint", this::getSetpoint, this::setSetpoint);
    builder.addDoubleProperty("measurement", this::getMeasurement, null);
    builder.addDoubleProperty("error", this::getError, null);
    builder.addBooleanProperty("atSetpoint", this::atSetpoint, null);
  }

}

/*
Look at downloads\tbh-and-bang-bang-speed-control.pdf for Ether voltage initialization.

Take-Back Half
Theory
Take Back Half is essentially an Integrator (the I portion of a PID Controller),
but every time that the error crosses zero, the output is cut. This makes it a
good option for use in velocity control, and it gained popularity in the 2015-16
Nothing But Net season of VRC.

A more in depth look at Take Back Half can be found in this forum thread: Flywheel Velocity Control

Advantages
Only one gain constant to tune - easier than PID
Fast Response Time
Decent stability except with exceedingly high gain
Disadvantages
Can only be used in velocity control applications
Example
After initializing the tbh, initial output, and gain parameters:

e = S-P;                            // calculate the error;
Y += G*e;                           // integrate the output;
if (Y>1) Y=1; else if (Y<0) Y=0;    // clamp the output to 0..+1;
if (signbit(e)!=signbit(d)){        // if zero crossing,
  Y = b = 0.5*(Y+b);                // then Take Back Half
  d = e;}                           // and save the previous error;
…where:

S is the setpoint (target RPM)
P is the process variable (measured RPM)
G is the integral gain (the tuning parameter)
Y is the output command to the motor controller
e is the error
d is the previous error
b is the TBH variable

Y, d, and b should be initialized.

Would someone be willing to test this and make any necessary corrections and re-post
 for the benefit of C language teams who might want to try TBH?

1 Reply

Ether
Mar '13
Y, d, and b should be initialized.

^Following up on the above^

You can improve the spinup response of TBH by clever selection of the initial values
 before you change the setpoint "S" from 0 to your desired speed.

Do a simple open-loop test to establish the approximate value of motor command
 (in the range 0 to +1) required to hold your wheel speed at the target value.
  Call this experimentally determined motor command value "M". It doesn’t have to be exact.

To start your spinup do the following:

Set S to your desired wheel speed; initialize Y=1, d=1, and b=2*M-1; and turn your speed
 controller on.

Since Y=1, you will be applying full voltage to the motor to spin it up (just like bang-bang).
 Y will remain equal to 1 (applying full voltage) during the spinup, because there will be no
  zero crossings until you reach the target speed S.

When you reach the first zero crossing (at the target speed), the TBH algorithm will set
 Y (and b) equal to (Y+b)/2 = (1+(2*M-1))/2 = M, which is the experimentally-determined motor
  command value required to maintain the wheel at the target speed. You will immediately have
   the correct (or approximately correct) motor command for your target speed. This will
    reduce overshoot and oscillation.
*/