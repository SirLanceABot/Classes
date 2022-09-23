package frc.robot;

import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;

/**
 * This is a sample program to demonstrate the use of a Bang-Bang controller
 * with an ultrasonic sensor to reach and maintain a set distance from an object.
 */
public class Robot extends TimedRobot {
 
  private static final double kHoldSpeed = 100.0; // speed to hold (velocity setpoint)

  private static final int kLeftMotorPort = 3; // SparkMax CAN id

  private final BangBangController m_bangbangController = new BangBangController();

  private double gain = 1./5700.; // controller output of 1. ==> max rpm of 5700  good first guess of the gain for bare Neo
  private final TBHController m_tbhController = new TBHController(gain, 0.05); // 0.0002;  1.0 controller output ==> max rpm

  private static final CANSparkMax leftMotor = new CANSparkMax(kLeftMotorPort, com.revrobotics.CANSparkMaxLowLevel.MotorType.kBrushless);
  private RelativeEncoder m_leftEncoder = leftMotor.getEncoder();
 
  private Timer timer = new Timer();

  @Override
  public void robotInit()
  {
    // LiveWindow.disableAllTelemetry() to cut it out completely here or in Robot constructor?

    //Initialize Neo on a SparkMax
    leftMotor.restoreFactoryDefaults();
    leftMotor.setIdleMode(IdleMode.kCoast);
    // none of these faster settings seems to work - data still slow to update; don't know why
    leftMotor.setControlFramePeriodMs(5);
    leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 5); // 10
    leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 5); // 20
    leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 5); // 50
    leftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 5); // not used?
    m_leftEncoder.setMeasurementPeriod(5); // not used if brushless
    leftMotor.set(0);

    System.out.println(System.getenv()); // get all environmentals
    System.out.println(System.getenv("serialnum")); // get the roboRIO serial number

    // get the roboRIO comment is illustrative and has nothing to do with the Bang-Bang or TBH controllers
    //// start get roboRIO comment
    final Path commentPath = Path.of("/etc/machine-info");
    try {  
      // var temp = System.currentTimeMillis() + "," + (Runtime.getRuntime().totalMemory() - Runtime.getRuntime().freeMemory() + "\n");
      // Files.writeString(memoryLog, temp, StandardCharsets.UTF_8, StandardOpenOption.CREATE, StandardOpenOption.APPEND);
      var comment = Files.readString(commentPath);
      System.out.println(comment);
    } catch (IOException e) {
      // Couldn't read the file -- handle it how you want
      System.out.println(e);
    }
    //// end get roboRIO comment
  }

  @Override
  public void teleopInit() {
     m_leftEncoder.setPosition(0.);
    Timer.delay(.5); // wait for setPosition to finish; maybe shorted would still always work
      
    System.out.println("end of teleopInit");
    timer.reset();
    timer.start();
  }

  @Override
  public void teleopPeriodic() {
    // returned value is filtered with a rolling median filter
    // var speed = m_filter.calculate(m_leftEncoder.getVelocity());
    var speed = m_leftEncoder.getVelocity();
    var position = m_leftEncoder.getPosition();

    if(timer.hasElapsed(3.)) // give teleop mode a few seconds to settle before looking closely at the data
    {
      System.out.println("wait over; applying TBH control");
      timer.stop();
      timer.reset();
    // Set setpoint of the controller
    m_bangbangController.setTolerance(10.); // give it a little leeway; this is in units of setpoint so should update if setpoint changed
    m_bangbangController.setSetpoint(kHoldSpeed);

    m_tbhController.setSetpoint(kHoldSpeed, kHoldSpeed*gain); // good first guess of the "feed forward"
    }
    // bang bang is all (below setpoint it's a 1.0) or nothing (above setpoint it's a 0.0)
    double bangbangOutput = m_bangbangController.calculate(speed);
    System.out.println("BangBang " + speed + " " + bangbangOutput + " " + m_leftEncoder.getVelocity());
    double maxSpeed = 1.0;
    // leftMotor.set(bangbangOutput == 0.? 0.0 : maxSpeed);
    // if(leftMotor.getLastError() != REVLibError.kOk) System.out.print("set " + leftMotor.getLastError());

    // use TBH take back half controller
    var TBHoutput = m_tbhController.calculate(speed);
    leftMotor.set(TBHoutput);
    if(leftMotor.getLastError() != REVLibError.kOk) System.out.print("set " + leftMotor.getLastError());
    System.out.println("TBH " + speed + " " + TBHoutput + " " + position);
  }

  @Override
  public void testInit()
  {
    teleopInit();
  }

  @Override
  public void testPeriodic()
  {
    LiveWindow.updateValues();
    teleopPeriodic();
  }

}
