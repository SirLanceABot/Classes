package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.PID_ATune.CONTROL_TYPE;
import frc.robot.PID_ATune.DIRECTION;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;

public class Robot extends TimedRobot {
  private static int sampleTime = 4; // milliseconds loop sample time period
  private static TalonSRX Motor;
  private PID_ATune tuner;
  private long millisStart;
  // control (output) signal limits
  double controllerMin = -1., controllerMax = 1.;
  double output= 0.2; // %vBus for Talon
  double oStep = 0.1; // + and - step size for the output perturbation (relay)
  double setpoint; // below we'll find out the rpm we get for the output signal specified above
  double RPMconversion = 1.116071428571429; // convert raw encoder to gear box shaft RPM  * (10.* 60.)/(19.2 * 7. * 4.) = 1.116071428571429
  boolean tuning = true;
  StripChart myChart;

  Robot()
  {
    super(.005); // set the robot loop time
  }

  @Override
  public void robotInit()
  {
   Motor = new TalonSRX(0);
   Motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);
   Motor.setStatusFramePeriod(2, 5, 30);
  }

  @Override
  public void teleopInit() {

    // Tuned with %VBus/RPM of gearbox shaft output

    Motor.set(ControlMode.PercentOutput, output);
      try {Thread.sleep(5000);} // let motor speed stabilize
          catch (InterruptedException e) {e.printStackTrace();}

    //get the (average) speed (setpoint or input) at this output (power level)

    setpoint = 0;
    for(int idx =1; idx <=20; idx++)
    {
        double speed = Motor.getSelectedSensorVelocity();
        setpoint += speed;
 
        //System.out.println("speed " + speed);
        try {Thread.sleep(100);} // let motor speed stabilize
          catch (InterruptedException e) {e.printStackTrace();}

    }

    setpoint /= 20.; // average of the raw sensor speed
    setpoint = setpoint * RPMconversion;
    System.out.println(" power level " + output + ", RPM " + setpoint + " power level step +- " + oStep);

    tuner = new PID_ATune( setpoint, output, oStep, DIRECTION.DIRECT, CONTROL_TYPE.PID_CONTROL, sampleTime);
    
    //   S E T U P   S T R I P C H A R T   T O   D I S P L A Y   A C T I O N S
    // center of left process variable graph; minimum value of right controller graph; maximum value of right controller graph
    myChart = new StripChart( setpoint,	output-oStep,	output+oStep);

    millisStart = System.currentTimeMillis(); // start time now will be relative 0;
  }

  @Override
  public void teleopPeriodic()
  {
    if(!tuning) return;

      double speed = Motor.getSelectedSensorVelocity() * RPMconversion; // get the speed of the current output; raw * conversion factor = rpm
      int loopStartTime = (int)(System.currentTimeMillis() - millisStart);
      int rc = tuner.Runtime(speed, loopStartTime);
      // 0 still looking for peaks
      // 1 stop - didn't find peaks; give up; or okay - peaks found and tuned
      // 2 waiting for time; no processing
      // 3 tuned but keep going
      //System.out.println("tuner ret " + rc);

      switch (rc) // 0 took a time step; 1 done tuning; 2 called faster than sample rate; 3 intermediate parameters available
      {
      case 1:  // tuning just completed; mark that event, print the peak, and move on
        tuning = false;
      case 3:  // time step okay, print the peak, and process time step
        System.out.format("  peaks %d-%d, Ku=%.5f, Pu=%.5f, Kp=%.5f, Ki=%.5f, Kd=%.5f %%VBus/RPM s\n", 
          (int)(1000.*tuner.GetPeak_1()), (int)(1000.*tuner.GetPeak_2()), tuner.GetKu(), tuner.GetPu(), tuner.GetKp(), tuner.GetKi(), tuner.GetKd());
// Ku ultimate gain; Pu ultimate period
//////////////////////////////////////////////////////////////////////////////////
//            START Conversion to Talon units
//
// Tuned with %VBus/RPM
// Talon SRX PID controller internally uses throttle units/encoder edges per 100ms
// Essentially undo the units conversion previously done
//
//           1023 throttle units per %VBus / (4 edges per quadrature encoder pulse * 7 encoder pulses per motor rev
//           * 19.2 motor revs per gear box output shaft rev /
//           (60 seconds per minute * 10 100ms per seconds))
//				double KuUnits = 1023. / (( 4. * 7. * 19.2 ) / ( 60. * 10. ));
        double KuUnits = 1023. * RPMconversion;
// Tuned with Seconds;  Talon SRX PID controller internally uses milliseconds for intgeration and differentiation
//           1000 milliseconds per seconds
				double PuUnits = 1000.;

				double kP = tuner.GetKp() * KuUnits;
				double kI = tuner.GetKi() * KuUnits/PuUnits;
				double kD = tuner.GetKd() * KuUnits*PuUnits;
				System.out.println("Talon PID Kp= " + kP + " Ki=" + kI + " Kd=" + kD + " throttle units/edges/motor shaft rev/100ms ms\n");
//
//            END Conversion to CAN Talon units
/////////////////////////////////////////////////////////////////////////////////

      case 0:  // time step okay, process time step
        break;
      case 2:  // too fast, skipping this step
        return;
      default:
        System.err.println("\n\nUnknown return from Runtime()\n\n");
      }

      output = tuner.getOutput(); // get the new output
      Motor.set(ControlMode.PercentOutput, output); // set a new speed using the new output

      // time in milliseconds;	process variable - input to controller;	output from controller
      System.out.print("\n" + myChart.PrintStripChart( loopStartTime,	speed, output ) );
      //System.out.println("speed " + speed + ", output " + output);

      if(!tuning)
      {
        Motor.set(ControlMode.PercentOutput, 0.); // stop
      }
  }
}

       // .2 %Vbus is about 60.268 rpm is about 54 raw units  1.116071428571429
        // getVelocity is raw encoder per 100ms
        // *10 100ms/s
        // *60 s/min
        // /19.2 motor revs/gear box output rev
        // /7 encoder pulses/ motor rev
        // /4 edges /pulse
    //    Encoder:
//There is an encoder mounted to the back side of this motor.
//  It is a 7 pulse per revolution (ppr) hall effect encoder.
//   Since the motor's gearbox has a 19.2:1 reduction,
//    the NeverRest Orbital 20 output shaft provides 134.4 ppr, or 537.6 counts per revolution within the software environment.
        // neverest 20:1



/*
// Tuned with %VBus/encoder edges per 100ms
// Talon SRX PID controller internally uses throttle units/encoder edges per 100ms
//
//           1023 throttle units per %VBus
				double KuUnits = 1023.;
//
// Tuned with Seconds;  Talon SRX PID controller internally uses milliseconds
//           1000 milliseconds per seconds
				double PuUnits = 1000.;
*/