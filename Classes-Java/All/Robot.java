package frc.robot;

import static java.lang.System.*;
import java.util.function.Function;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.hal.PWMConfigDataResult;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalOutput;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.Watchdog;

import com.kauailabs.navx.frc.AHRS;
import com.kauailabs.navx.frc.AHRS.SerialDataType;

// Import material to implement Talons and controls.
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;

public class Robot extends TimedRobot {

static {out.println("Entered robot static class initializer");}
{out.println("Entered robot non-static instance initializer");}

public final static class Constants {

	public final static class LIDAR {
		public static final I2C.Port PORT = I2C.Port.kMXP /*I2C.Port.kOnboard*/;  // select roboRIO I2C port
		public static final LIDAR_Lite.Address ADDRESS = LIDAR_Lite.Address.DEFAULT;	// I2C address on selected port
		private LIDAR() {}// restrict instantiation with a private constructor - no instantiation is needed for just static constants
	}
	private Constants() {}// restrict instantiation with a private constructor - no instantiation is needed for just static constants
}

  //////////// BuiltInAccelerometer ////////////
  private final BuiltInAccelerometer accel = new BuiltInAccelerometer();

  //////////// JOYSTICK INPUT/OUTPUT /////////
  private final Joystick m_stick = new Joystick(0);

  private final XboxController m_hid = new XboxController(0);
  Function<XboxController, Boolean> doFuncXX = (controller) -> controller.getAButton();
  Function<XboxController, Boolean> doFuncYY = (controller) -> controller.getBButton();

  //////////// RELAY OUTPUT ////////////
  private final Relay m0_relay = new Relay(0);

  boolean flip = false;

  //////////// ANALOG INPUT /////////////
  //
  // do all 4 onboard inputs (ports 0 to 3) to see if there is any cross-talk
  private static AnalogInput[] ai = new AnalogInput[4];
  {
    for (int idx = 0; idx < ai.length; idx++) {
      ai[idx] = new AnalogInput(idx); // array index and port number the same
    }
  }

  //////////// DIGITAL INPUT ////////////
  //
  // Digital inputs on the roboRIO are pulled high (3.3 v) when open circuit thus reading 1 or true.
  // Short the S (Signal) pin to ground to pull it down to reading 0 or false.
  // For passive mechanical switches there is no need to use the 5 volt pin.
  // Connecting 5 Volt to ground would be a dead short.
  // The 5 volt pin for digitals is to power active switches such as photoeyes.
  // For active devices choose NPN mode for best compatibility with the roboRIO.

  // Define counters to see how clean is the make/break of the switch (debounce) and
  // make sure limit wasn't exceeded.  If no hard-stop on mechanical travel, the travel
  // might have gone past the switch causing "reset" to not past limit.
  // WPILib recently included a debounce filter - try it out!

  private final int diPort = 9;
  DigitalInput di = new DigitalInput(diPort);

  private final int microNOPort = 1;
  private final int microNCPort = 2;
  private final int NO = 0; // subscript to access microSwitchCounter array
  private final int NC = 1; // subscript to access microSwitchCounter array
  private final DigitalInput[] microSwitch = { new DigitalInput(microNOPort), new DigitalInput(microNCPort) };
  private final Counter[] microSwitchCounter = { new Counter(microSwitch[NO]), new Counter(microSwitch[NC]) };

  //////////// DIGITAL OUTPUT /////////////
  private final int doPort = 8;
  DigitalOutput dO = new DigitalOutput(doPort);

  //////////// NAVX GYRO INPUT ////////////
  AHRS ahrs;
  float prevTempC; // for analyzing any temperature/drift relationship

  ///////////// TIMER //////////////
  Timer timer = new Timer(); // for blinking the LED light ring on the relay

  //////////// AMS COLOR SENSOR (Rev Robotics Color Sensor v1) //////////////
  private final AMSColorSensor colorSensor = new AMSColorSensor(AMSColorSensor.Constants.ONBOARD,
      AMSColorSensor.Constants.DEFAULT_ADDRESS);
  
  ///////// LIDAR LITE //////////////////
  private final static LIDAR_Lite lidar = new LIDAR_Lite(Constants.LIDAR.PORT, Constants.LIDAR.ADDRESS);

  ////////// TALON MOTOR CONTROLLER /////////////
  private final static TalonSRX leftMotor = new TalonSRX(0);
  private final static TalonSRX rightMotor = new TalonSRX(1);
  private final static TalonFX  FXmotor = new TalonFX(2);
  // static TalonSRX leftMotor;
  // static TalonSRX rightMotor;
  // static TalonFX  FXmotor;

  private double maxSpeed = 0.15; // must be small if using 120v power supply

  ///////// SERVO /////////////
  private final static Servo servo = new Servo(0);

  ///////// WATCHDOG /////////
  // Define what will be an interrupt handler
  // This line with lambda expression
  private final Watchdog puppy = new Watchdog(0.01, () -> {out.println("Watchdog barked");});
  // OR all this stuff is exactly the same
  // public class myPuppyInterrupt implements Runnable {
  // public void run() {
  // out.println("Watchdog barked");
  // }
  // }
  // myPuppyInterrupt puppyCallback = new myPuppyInterrupt();
  // private Watchdog puppy = new Watchdog(0.01, puppyCallback);
  
  ///////// end WATCHDOG //////////

  int count = 0;
  
  @Override
  public void robotInit() {

    out.println("Entered robotInit");

    // get the "permanently" stored preferences (from roboRIO lvuser's networktables.ini)
    // smartdashboard can manipulate these preferences and save or load them on the PC, too
    // there is a backup (previous version) on the roborRIO networktables.ini.bak

    System.out.println("All preferences\n" + Preferences.getKeys());
    // Preferences.putInt("FLencoder", 123);
    // Preferences.putInt("FRencoder", 234);
    // Preferences.putInt("BLencoder", 345);
    // Preferences.putInt("BRencoder", 456);
    if(Preferences.containsKey("FLencoder"))
    {
      System.out.println("Using FLencoder position " + Preferences.getInt("FLencoder", 0));
    }
    else
    {
      DriverStation.reportError("Missing FLencoder position", false);
    }
  
    //configTalon();

    m0_relay.setDirection(Relay.Direction.kForward);

    for (int idx = 0; idx < ai.length; idx++) {
      SmartDashboard.putNumber("Analog Input " + idx, 0.);
    }

    dO.set(true); // initialize otherwise it's "random" value

    initNavX();

    initDrive();

    microSwitchCounter[NO].reset();
    microSwitchCounter[NC].reset();

    timer.start();

    puppy.suppressTimeoutMessage(true); // if you don't want the standard message to System.out
    puppy.enable();
    // puppy.reset(); // do this somewhere in code to indicate program flowed there or not
  
    out.println("Exiting robotInit");
    out.flush();
  }

  @Override
  public void robotPeriodic() {

    displayLIDAR();

    displayColor();

    blinkLightRingRelay();

    displayRobotController();

    displayAccelerometer();

    displayAnalog(); // if 3 isn't connected it gets cross-talk from 2

    displayMicroSwitch();

    out.println("do Port " + doPort + " " + dO.get());

    displayNavX();
  }

  @Override
  public void disabledInit() {
    // Stop the rumble when entering disabled
    m_hid.setRumble(RumbleType.kLeftRumble, 0.0);
    m_hid.setRumble(RumbleType.kRightRumble, 0.0);
  }

  @Override
  public void disabledPeriodic() {
    super.disabledPeriodic();
  }

  @Override
  public void autonomousInit() {
    // Turn on rumble at the start of auto
    m_hid.setRumble(RumbleType.kLeftRumble, 1.0);
    m_hid.setRumble(RumbleType.kRightRumble, 1.0);
  }

  @Override
  public void autonomousPeriodic() {
    super.autonomousPeriodic();
  }

  @Override
  public void teleopInit() {
    initDrive();
  }

  @Override
  public void teleopPeriodic() {
    /*
     * Retrieve the button values. GetRawButton will return true if the button is
     * pressed and false if not.
     */
    displayXbox();

    setDisplayServo(0.5); // refresh the servo position

    drive();
  }

  @Override
  public void testInit() {
    super.testInit();
  }

  @Override
  public void testPeriodic() {
    super.testPeriodic();
  }

  //////////////////////////////////////////////////////
  //////////////////////////////////////////////////////

  private void initDrive() {
    leftMotor.configFactoryDefault();
    rightMotor.configFactoryDefault();

    leftMotor.setNeutralMode(NeutralMode.Brake);
    rightMotor.setNeutralMode(NeutralMode.Brake);

    leftMotor.setInverted(true);
    rightMotor.setInverted(false);

    leftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);
    rightMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);

    leftMotor.setSelectedSensorPosition(0);
    rightMotor.setSelectedSensorPosition(0);

    rightMotor.follow(leftMotor);
  }

  private void drive() {
    // robot must be enabled to run motors

    double driveSpeed=maxSpeed; // start running full speed in this direction; ok for the test tub

    if (leftMotor.getSensorCollection().isRevLimitSwitchClosed()) {
        driveSpeed = maxSpeed; // go away from limt switch
    }
    else
    if (leftMotor.getSensorCollection().isFwdLimitSwitchClosed()) {
        driveSpeed = -maxSpeed; // go away from limt switch
    }

    leftMotor.set(ControlMode.PercentOutput, driveSpeed);

    out.print(
      "motors currents " + leftMotor.getSupplyCurrent() + ", " + leftMotor.getSupplyCurrent());
    
    out.println(
        ", positions " + leftMotor.getSelectedSensorPosition(0) + ", " + rightMotor.getSelectedSensorPosition(0));
  }

  private void displayXbox() {
    out.println("Xbox buttons " + doFuncXX.apply(m_hid) + "  " + doFuncYY.apply(m_hid));
  }

  private void displayAccelerometer() {
    out.println("BuiltInAccelerometer x, y, z " +
      accel.getX() + " " + accel.getY() + " " + accel.getZ());
  }

  private void displayAnalog() {
    for (int idx = 0; idx < ai.length; idx++) {
      // out.println("Analog Input Voltage " + ai[idx].getAverageVoltage());
      SmartDashboard.putNumber("Analog Input " + idx, ai[idx].getAverageVoltage());
    }
  }

  private void displayColor() {
    final Colors crgb = new Colors();
    colorSensor.get(crgb);
    out.println(crgb);
  }

  private void displayLIDAR() {
    out.println(lidar);
  }

  private void blinkLightRingRelay() {
    if (timer.advanceIfElapsed(2.0)) {
      // relay controlled in this robotPeriodic but
      // robot has to be enabled for relays and any motrs to be on
      // likely that means PWM, too. Test that.
      flip = !flip;
      out.println("FLIP = " + flip);
      if (isDisabled())
        out.println(" Disabled mode - any relays are off");
      if (flip) {
        m0_relay.set(Relay.Value.kOn); // forward and reverse on
      } else {
        m0_relay.set(Relay.Value.kOff); // forward and reverse off
      }
    }
  }

  private void displayRobotController() {
    System.out.format("%d %b %4.1f %4.1f %5.3f %4.1f %5.3f %4.1f %5.3f %4.1f %5.5f\n", RobotController.getFPGATime(), // long
        RobotController.getUserButton(), // boolean

        RobotController.getBatteryVoltage(),

        RobotController.getInputVoltage(), RobotController.getInputCurrent(),

        RobotController.getVoltage3V3(), RobotController.getCurrent3V3(),

        RobotController.getVoltage5V(), RobotController.getCurrent5V(),

        RobotController.getVoltage6V(), RobotController.getCurrent6V());
  }

  private void displayMicroSwitch() {
    out.println(
        "micro NO Port " + microNOPort + " " + microSwitch[NO].get() + " counter " + microSwitchCounter[NO].get());

    out.println(
        "micro NC Port " + microNCPort + " " + microSwitch[NC].get() + " counter " + microSwitchCounter[NC].get());
    }

  private void setDisplayServo(final double position) {
        // too much output for servo that doesn't much change
        PWMConfigDataResult a;

        a = servo.getRawBounds();
        out.format("%d %d %d %d %d\n",
          a.center,
          a.deadbandMax,
          a.deadbandMin,
          a.max,
          a.min);
    
        servo.set(position);
      
        a = servo.getRawBounds();
        out.format("%d %d %d %d %d\n",
          a.center,
          a.deadbandMax,
          a.deadbandMin,
          a.max,
          a.min);
    
        out.println("Servo raw " + servo.getRaw());
  }

  private void configTalon()
  {
      leftMotor.set(ControlMode.PercentOutput, 0.0d);
      rightMotor.set(ControlMode.PercentOutput, 0.0d);

      leftMotor.configFactoryDefault();
      rightMotor.configFactoryDefault();

      leftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 10);
      leftMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 1);
      //configNeutraldeadband(0.001, 
      
      leftMotor.setInverted(true);
      rightMotor.setInverted(false);

      rightMotor.follow(leftMotor);

      leftMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 30);
      //masterMotor.configSelectedFeedbackSensor(FeedbackDevice.Analog);
      leftMotor.setSelectedSensorPosition(0);
      //leftMotor.setInverted(InvertType.InvertMotorOutput);
      leftMotor.setSensorPhase(false);

      leftMotor.configPeakCurrentLimit(3);
      leftMotor.configPeakCurrentDuration(200);
      leftMotor.configContinuousCurrentLimit(2);
      leftMotor.enableCurrentLimit(true);

      leftMotor.configOpenloopRamp(0.);
      //masterMotor.configReverseSoftLimitThreshold(getArmPositionPotValue(ArmPosition.kTopArmPosition));
      //masterMotor.configForwardSoftLimitThreshold(getArmPositionPotValue(ArmPosition.kFloorArmPosition));
      leftMotor.configForwardSoftLimitEnable(false);
      leftMotor.configReverseSoftLimitEnable(false);
      //masterMotor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled, 0);
      leftMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
      leftMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0);
      
      leftMotor.configSetParameter(ParamEnum.eFeedbackNotContinuous, 1,0,0,0);
      out.print(leftMotor.getSensorCollection().getQuadraturePosition());
      out.println("left motor velocity " + leftMotor.getSelectedSensorVelocity());
      out.println("FWD limit switch is: " + leftMotor.getSensorCollection().isFwdLimitSwitchClosed() +
      " AnalogIn is: " + leftMotor.getSensorCollection().getAnalogInRaw() +
      " AnalogIn Position is: " + leftMotor.getSelectedSensorPosition());

      // Used for a Limit Switch that is attached to a Talon but is not controlling the Talon output
      //leftMotor.configForwardLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
      // masterMotor.configReverseLimitSwitchSource(LimitSwitchSource.Deactivated, LimitSwitchNormal.Disabled);
       
      leftMotor.configSetParameter(ParamEnum.eFeedbackNotContinuous, 1, 0, 0, 0);

      leftMotor.overrideSoftLimitsEnable(false);
}

private void initNavX() {
  try {
    /***********************************************************************
     * navX-MXP: - Communication via RoboRIO MXP (SPI, I2C, TTL UART) and USB. - See
     * http://navx-mxp.kauailabs.com/guidance/selecting-an-interface.
     * 
     * navX-Micro: - Communication via I2C (RoboRIO MXP or Onboard) and USB. - See
     * http://navx-micro.kauailabs.com/guidance/selecting-an-interface.
     * 
     * Multiple navX-model devices on a single robot are supported.
     ************************************************************************/
    // ahrs = new AHRS(SerialPort.Port.kUSB1);
    ahrs = new AHRS(SerialPort.Port.kUSB1, SerialDataType.kProcessedData, (byte) 200);
    Timer.delay(1.0); // make sure AHRS USB communication is done before doing
    // other USB. Also give it time to settle in; a few more seconds never hurt
    ahrs.enableLogging(true);
    Shuffleboard.getTab("Gyroscope").add((Sendable) ahrs);
    out.println("NavX update " + ahrs.getActualUpdateRate() + " " + ahrs.getUpdateCount() + " " + ahrs.getRequestedUpdateRate());
    // ahrs = new AHRS(SerialPort.Port.kMXP, SerialDataType.kProcessedData, (byte) 200);
    // ahrs = new AHRS(SPI.Port.kMXP);
    // ahrs = new AHRS(I2C.Port.kMXP);
    // ahrs = new AHRS(I2C.Port.kOnboard, (byte) 60);
    out.println("NavX update " + ahrs.getActualUpdateRate() + " " + ahrs.getUpdateCount() + " " + ahrs.getRequestedUpdateRate());
  } catch (final RuntimeException ex) {
    DriverStation.reportError("Error instantiating navX:  " + ex.getMessage(), true);
  }

  ahrs.zeroYaw();
  prevTempC = ahrs.getTempC(); // for watch temperature stablize
}

private void displayNavX()
{
  {
    final float RobotHeading = ahrs.getYaw();
    SmartDashboard.putNumber("IMU_Yaw", RobotHeading/* ahrs.getYaw() */);
    SmartDashboard.putNumber("IMU_Pitch", ahrs.getPitch());
    SmartDashboard.putNumber("IMU_Roll", ahrs.getRoll());
    SmartDashboard.putString("Rotation2d", ahrs.getRotation2d().toString());
    SmartDashboard.putNumber("Rotation2d radians", ahrs.getRotation2d().getRadians());
    

    /* Display tilt-corrected, Magnetometer-based heading (requires */
    /* magnetometer calibration to be useful) */
    SmartDashboard.putBoolean("Magnetic Disturbance", ahrs.isMagneticDisturbance());
    SmartDashboard.putBoolean("Magnetometer Calibrated", ahrs.isMagnetometerCalibrated());

    SmartDashboard.putNumber("IMU_CompassHeading", ahrs.getCompassHeading());

    /* Display 9-axis Heading (requires magnetometer calibration to be useful) */
    SmartDashboard.putNumber("IMU_FusedHeading", ahrs.getFusedHeading());

    /* These functions are compatible w/the WPI Gyro Class, providing a simple */
    /* path for upgrading from the Kit-of-Parts gyro to the navx MXP */

    SmartDashboard.putNumber("IMU_TotalYaw", ahrs.getAngle());
    SmartDashboard.putNumber("IMU_YawRateDPS", ahrs.getRate());

    /* Display Processed Acceleration Data (Linear Acceleration, Motion Detect) */

    SmartDashboard.putNumber("IMU_Accel_X", ahrs.getWorldLinearAccelX());
    SmartDashboard.putNumber("IMU_Accel_Y", ahrs.getWorldLinearAccelY());
    SmartDashboard.putNumber("IMU_Accel_Z", ahrs.getWorldLinearAccelZ());
    SmartDashboard.putBoolean("IMU_IsMoving", ahrs.isMoving());
    SmartDashboard.putBoolean("IMU_IsRotating", ahrs.isRotating());

    /* Display estimates of velocity/displacement. Note that these values are */
    /* not expected to be accurate enough for estimating robot position on a */
    /* FIRST FRC Robotics Field, due to accelerometer noise and the compounding */
    /* of these errors due to single (velocity) integration and especially */
    /* double (displacement) integration. */

    SmartDashboard.putNumber("Velocity_X", ahrs.getVelocityX());
    SmartDashboard.putNumber("Velocity_Y", ahrs.getVelocityY());
    SmartDashboard.putNumber("Velocity_Z", ahrs.getVelocityZ());
    SmartDashboard.putNumber("Displacement_X", ahrs.getDisplacementX());
    SmartDashboard.putNumber("Displacement_Y", ahrs.getDisplacementY());
    SmartDashboard.putNumber("Displacement_Z", ahrs.getDisplacementZ());

    /* Display Raw Gyro/Accelerometer/Magnetometer Values */
    /* NOTE: These values are not normally necessary, but are made available */
    /* for advanced users. Before using this data, please consider whether */
    /* the processed data (see above) will suit your needs. */

    SmartDashboard.putNumber("RawGyro_X", ahrs.getRawGyroX());
    SmartDashboard.putNumber("RawGyro_Y", ahrs.getRawGyroY());
    SmartDashboard.putNumber("RawGyro_Z", ahrs.getRawGyroZ());
    SmartDashboard.putNumber("RawAccel_X", ahrs.getRawAccelX());
    SmartDashboard.putNumber("RawAccel_Y", ahrs.getRawAccelY());
    SmartDashboard.putNumber("RawAccel_Z", ahrs.getRawAccelZ());
    SmartDashboard.putNumber("RawMag_X", ahrs.getRawMagX());
    SmartDashboard.putNumber("RawMag_Y", ahrs.getRawMagY());
    SmartDashboard.putNumber("RawMag_Z", ahrs.getRawMagZ());
    var TempC = ahrs.getTempC();
    var deltaTempC =TempC - prevTempC;
    prevTempC = TempC;
    SmartDashboard.putNumber("IMU_Temp_C", TempC);
    SmartDashboard.putNumber("IMU_Temp_C_delta", deltaTempC);
//    SmartDashboard.putNumber("IMU_Temp_C", ahrs.getTempC());
    SmartDashboard.putNumber("IMU_Timestamp", ahrs.getLastSensorTimestamp());

    SmartDashboard.putNumber("Fused Heading", ahrs.getFusedHeading());

    /* Omnimount Yaw Axis Information */
    /* For more info, see http://navx-mxp.kauailabs.com/installation/omnimount */
    final AHRS.BoardYawAxis yaw_axis = ahrs.getBoardYawAxis();
    SmartDashboard.putString("YawAxisDirection", yaw_axis.up ? "Up" : "Down");
    SmartDashboard.putNumber("YawAxis", yaw_axis.board_axis.getValue());
    // out.println(AHRS.BoardAxis.kBoardAxisX + " " +
    // AHRS.BoardAxis.kBoardAxisY + " " + AHRS.BoardAxis.kBoardAxisZ +
    // yaw_axis.board_axis.getValue());
    final String kBoardAxisAlpha[] = { "BoardAxisX", "BoardAxisY", "BoardAxisZ" };
    SmartDashboard.putString("YawAxisAlpha", kBoardAxisAlpha[yaw_axis.board_axis.getValue()]);

    /* Sensor Board Information */
    SmartDashboard.putString("FirmwareVersion", ahrs.getFirmwareVersion());

    /* Quaternion Data */
    /* Quaternions are fascinating, and are the most compact representation of */
    /* orientation data. All of the Yaw, Pitch and Roll Values can be derived */
    /* from the Quaternions. If interested in motion processing, knowledge of */
    /* Quaternions is highly recommended. */
    SmartDashboard.putNumber("QuaternionW", ahrs.getQuaternionW());
    SmartDashboard.putNumber("QuaternionX", ahrs.getQuaternionX());
    SmartDashboard.putNumber("QuaternionY", ahrs.getQuaternionY());
    SmartDashboard.putNumber("QuaternionZ", ahrs.getQuaternionZ());

    /* Connectivity Debugging Support */
    SmartDashboard.putNumber("IMU_Byte_Count", ahrs.getByteCount());
    SmartDashboard.putNumber("IMU_Update_Count", ahrs.getUpdateCount());
    // if (ahrs->IsAltitudeValid()) // Aero only
    // {
    // SmartDashboard::PutNumber( "Barometric Pressure",
    // ahrs->GetBarometricPressure() );
    // SmartDashboard::PutNumber( "Altitude", ahrs->GetAltitude() );
    // SmartDashboard::PutNumber( "Pressure", ahrs->GetPressure() );
    // }
    // else
    // {
    // SmartDashboard::PutString( "Barometric Pressure", (llvm::StringRef)"Not
    // Available" );
    // SmartDashboard::PutString( "Altitude", (llvm::StringRef)"Not Available" );
    // SmartDashboard::PutString( "Pressure", (llvm::StringRef)"Not Available" );
    // }

    /* Display 6-axis Processed Angle Data */
    SmartDashboard.putBoolean("IMU_Connected", ahrs.isConnected());
    SmartDashboard.putBoolean("IMU_IsCalibrating", ahrs.isCalibrating());
  }
}
}

/*
package frc.robot;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;


public class Robot extends TimedRobot {
  // distance in inches the robot wants to stay from an object
  private static final double kHoldDistance = 12.0;

  // factor to convert sensor values to a distance in inches
  private static final double kValueToInches = 0.125;

  // proportional speed constant
  private static final double kP = 0.05;

  private static final int kLeftMotorPort = 0;
  private static final int kRightMotorPort = 1;
  private static final int kUltrasonicPort = 0;

  // median filter to discard outliers; filters over 10 samples
  private final MedianFilter m_filter = new MedianFilter(10);

  private final AnalogInput m_ultrasonic = new AnalogInput(kUltrasonicPort);
  private final DifferentialDrive m_robotDrive
      = new DifferentialDrive(new PWMVictorSPX(kLeftMotorPort),
      new PWMVictorSPX(kRightMotorPort));


  @Override
  public void teleopPeriodic() {
    // sensor returns a value from 0-4095 that is scaled to inches
    // returned value is filtered with a rolling median filter, since ultrasonics
    // tend to be quite noisy and susceptible to sudden outliers
    double currentDistance = m_filter.calculate(m_ultrasonic.getValue()) * kValueToInches;

    // convert distance error to a motor speed
    double currentSpeed = (kHoldDistance - currentDistance) * kP;

    // drive robot
    m_robotDrive.arcadeDrive(currentSpeed, 0);
  }
}

*/

