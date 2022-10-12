/** Sample PIDF controller in one CAN TalonFX motor controller using Integrated Sensor
 * 
 * Assumes a single rotational direction such as shooter flywheel
 * 
 * SET THE PARAMETERS AS YOU WISH
 * MOST PARAMETERS ARE MOST EASILY CHANGED USING THE PHOENIX TUNER
 * 
 * IN TELEOP MODE
 * --------------
 * All the PIDF constants and filter times are built into the code and are okay for no load
 * on a the Falcon 500 motor.
 * The intention is they can be changed in the Phoenix Tuner to tune a real device.
 * 
 * The setpoint speed is entered on the SmartDashboard as "velocity set (native units)".
 * It is the only input to the program.
 * Press TAB to have the input value sent (ENTER works but then the robot is disabled).
 * 
 * IN AUTO MODE
 * ------------
 * Programs runs fully automatically at several speeds to display the calculated kF.
 * Voltage compensation is included in the calculation as selected by the user.
 */

package frc.robot;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;

public class Robot extends TimedRobot {

/**
 * User settable parameters
 */
  final int flywheelMotorPort = 0;
  final double voltageCompensation = 10.; // if using voltageCompensation, kF should be determined with it on or kFcompensation = kFbattery * battery/voltageCompensation
  final double neutralDeadband = 0.001;
  final int pidIdx = 0; // Talon primary closed loop control (or none)
  final boolean invert = false;
  final int filterWindow = 1; // ms
  final SensorVelocityMeasPeriod filterPeriod = SensorVelocityMeasPeriod.Period_5Ms; // 10ms and 20ms had less fluctuation
  final int sampleTime = 15; // ms
  final double kP = 0.1; // works for the entire velocity range
  final double kI = 0.; // if used no effect then suddenly bad
  final double kD = 0.; // if used no effect then suddenly bad
  // final double kF = 0.046; // good around 8000 nu and okay for the entire velocity range with a little more error creeping in (~12.3v battery)
  final double kF = 0.0555; // okay for the entire velocity range with a little more error creeping in (10v compensation)
  final double integralZone = 0.; // no limit
  final double maxIntegralAccumulator = 0.; // no limit

  // TalonFX magic numbers
  double nativeToRPM = 10. * 60. / 2048.; // 10 .1sec/sec   60 secs/min   rev/2048 encoder ticks for Integrated Sensor
  double PctVBusToThrottle = 1023.; // 1023 talon throttle unit / 100%VBus

  // TalonFX flywheelMotorFollower;
  TalonFX flywheelMotor;
  private static final int TIMEOUT_MS = 50; // milliseconds TalonFX command timeout limit
  Runnable printSpeed;
  Consumer<Double> setFlywheelSpeed;
  Supplier<Double> getFlywheelSpeed;
  Supplier<Double> getSpeedError;
  Consumer<Double> setFlywheelPctVBus;
  Supplier<Double> getPctOutput;
  Supplier<Double> getBusVoltage;
  Supplier<Boolean> isVoltageCompensationEnabled;

  double speed = 0.; //initial speed to run and display on SmartDashboard
  // It seemed that sometimes a previous value from the SmartDashboard is used (race condition?).
  // This code tries hard to prevent but not sure it's perfect or what the issue was.
  int ParameterSetAttemptCount = 5; // retry flywheel config if error

  Robot()
  {
    LiveWindow.disableAllTelemetry(); // don't waste time on stuff we don't need
  }

  @Override
  public void robotInit() {
    
    PowerDistribution pd = new PowerDistribution();
    pd.clearStickyFaults();
    pd.close();

    SmartDashboard.putNumber("velocity set (native units)", speed);
    SmartDashboard.updateValues();

    createFlywheelMotorController(ParameterSetAttemptCount);
  
    Timer.delay(0.1); // let settle SmartDashboard updating and anything else
  }

  // calculate kF variables
  int count;
  double kfSpeed;
  double averageSpeed;
  double averagePctVoltage;
  double averageBusVoltage;

  @Override
  public void autonomousInit()
  {
    // clear accumulators and start at 0
    count = 0;
    kfSpeed = 0.;
    averageSpeed = 0.;
    averagePctVoltage = 0.;
    averageBusVoltage = 0.;
  }

  /**
   * print values related to kF calculation
   * collect data at 10 %VBus settings (and the 0 point with the NaNs)
   * skip 50 iterations to let speed settle
   * average values from the next 50 iterations
   */
  @Override
  public void autonomousPeriodic()
  {
    if(kfSpeed > 1.01)
    {
      setFlywheelPctVBus.accept(0.); // stop
      return;
    }

    count++;

    setFlywheelPctVBus.accept(kfSpeed);

    if(count <= 50) return; // skip 50 iterations
    // then gather data for 50
    averageSpeed += getFlywheelSpeed.get();

    averagePctVoltage += getPctOutput.get();

    averageBusVoltage += getBusVoltage.get();

    if(count < 100) return;
    // at iteration 100 print the smooth data and step up to next %VBus
    averageSpeed /= 50.;
    averagePctVoltage /= 50.;
    averageBusVoltage /= 50.;

    var kF = averagePctVoltage/averageSpeed * (isVoltageCompensationEnabled.get() ? averageBusVoltage / voltageCompensation : 1.) * PctVBusToThrottle; // voltage compensation correction factor

    System.out.format("%5.2f, %5.2f, %5.2f, %5.2f, kF=%7.4f\n", kfSpeed, averageSpeed, averagePctVoltage, averageBusVoltage, kF);

    kfSpeed += 0.1;
    count = 0;
    averageSpeed = 0.;
    averagePctVoltage = 0.;
  }

  @Override
  public void teleopInit()
  {
    SmartDashboard.putNumber("velocity set (native units)", speed); // in case the first failed try again - it might help
    SmartDashboard.updateValues();
    Timer.delay(0.1); // make sure velocity is set (again)
  }

  @Override
  public void teleopPeriodic() {
    speed = SmartDashboard.getNumber("velocity set (native units)", speed);
    setFlywheelSpeed.accept(speed);
    printSpeed.run();
  }


  /**
   * create the flywheel motor controller
   * 
   * @param attemptLimit configuration number of times if any errors (always run at least once)
   */
  public void createFlywheelMotorController(int attemptLimit)
  {
    flywheelMotor = new TalonFX(flywheelMotorPort);

    int setAttemptNumber = 0;
    
    // loop if not completed okay until retry limit
    while (! configFlywheelMotorController(flywheelMotorPort, voltageCompensation, neutralDeadband,
                                           pidIdx, invert, filterWindow, filterPeriod, sampleTime,
                                           kP, kI, kD, kF, integralZone, maxIntegralAccumulator) )
      {
        setAttemptNumber++;
        if (setAttemptNumber >= attemptLimit)
        {
          DriverStation.reportError("[Talon] failed to initialize flywheel motor controller on CAN id " + flywheelMotorPort, false);
          System.out.println("[Talon] failed to initialize flywheel motor controller on CAN id " + flywheelMotorPort);
          break;
        }
      }
  }


  /** Configure the Talon motor controller
   * 
   * @param flywheelMotorPort CAN
   * @param voltageCompensation limit motor input - 0. disables voltage compensation
   * @param neutralDeadband 0.001 to 0.25
   * @param pidIdx PID index 0 is either 0 or none
   * @param invert (motor reversed)
   * @param filterWindow ms
   * @param filterPeriod ms
   * @param sampleTime ms
   * @param kP
   * @param kI
   * @param kD
   * @param kF
   * @param integralZone integral zone (in native units) If the (absolute) closed-loop error is outside of this zone, integral accumulator is automatically cleared. This ensures than integral wind up events will stop after the sensor gets far enough from its target.
   * @param maxIntegralAccumulator Max integral accumulator (in native units)
   */
  boolean configFlywheelMotorController(int flywheelMotorPort, double voltageCompensation, double neutralDeadband, int pidIdx,  boolean invert,
                      int filterWindow, SensorVelocityMeasPeriod filterPeriod, int sampleTime,
                      double kP, double kI, double kD, double kF, double integralZone, double maxIntegralAccumulator)
  {
      int errors = 0; // count TalonFX method errors

      flywheelMotor.clearStickyFaults(TIMEOUT_MS);
      errors += check(flywheelMotor, "clear faults", true);

      flywheelMotor.configFactoryDefault(TIMEOUT_MS);
      errors += check(flywheelMotor, "set default", true);

      // flywheelMotorFollower = new TalonFX(1);
      // System.out.println("[Talon] clear faults " + flywheelMotorFollower.clearStickyFaults(TIMEOUT_MS));
      // System.out.println("[Talon] set default " + flywheelMotorFollower.configFactoryDefault(TIMEOUT_MS));
      // flywheelMotorFollower.follow(flywheelMotor);
  
      flywheelMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, sampleTime, TIMEOUT_MS);
      errors += check(flywheelMotor, "set status 2", true);
      flywheelMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, sampleTime, TIMEOUT_MS); // PID error
      errors += check(flywheelMotor, "set status 13", true);
      
      //System.out.println("[Talon] set status 10 " + flywheelMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, sampleTime, TIMEOUT_MS)); // this may or may not be useful

      flywheelMotor.setInverted(invert);
      errors += check(flywheelMotor, "set inverted", true);

      flywheelMotor.setNeutralMode(NeutralMode.Coast);
      errors += check(flywheelMotor, "set neutral mode", true);

      flywheelMotor.setSelectedSensorPosition(0, pidIdx, TIMEOUT_MS); // start at 0 position just for fun; not needed to tune velocity
      errors += check(flywheelMotor, "set sensor position", true);
     
      // get the factory defaults for some setting as defined by this Java API - may be different than Phoenix Tuner
			TalonFXConfiguration configs = new TalonFXConfiguration();

      // change the ones that we need to

      // voltage compensation seems potentially useful but not for sure to enable tuning at realistic voltage and for reproducibility
      // kF has to be increased by the amount of voltage reduction in the compensation. Check kP, too.
      configs.voltageCompSaturation = voltageCompensation;

      // one direction only assumed and forced - these work for both inverted or not
      configs.peakOutputReverse = 0.;
      configs.peakOutputForward = 1.;

      configs.neutralDeadband = neutralDeadband;

      configs.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

      configs.velocityMeasurementWindow = filterWindow;
      configs.velocityMeasurementPeriod = filterPeriod;
      configs.slot0.kP = kP;
      configs.slot0.kI = kI;
      configs.slot0.kD = kD;
      configs.slot0.kF = kF;
      configs.slot0.integralZone = integralZone;
      configs.slot0.maxIntegralAccumulator = maxIntegralAccumulator;

			flywheelMotor.configAllSettings(configs, TIMEOUT_MS); // send the new config back
      errors += check(flywheelMotor, "set configs", true);

      flywheelMotor.getAllConfigs(configs, TIMEOUT_MS); // read them back
      errors += check(flywheelMotor, "get configs", true);

      System.out.println("[Talon] configuration:\n" + configs); // print them

      if(voltageCompensation != 0.)
      {
        flywheelMotor.enableVoltageCompensation(true);
        errors += check(flywheelMotor, "set enable compensation error", true);
      }

      System.out.println("[Talon] compensation " + flywheelMotor.isVoltageCompensationEnabled());
      errors += check(flywheelMotor, "compensation error", true);

      System.out.println("[Talon] " + errors + " errors from config methods"); //TODO return the count or retry if not 0

      //
      // methods for others to access the TalonFX motor controller
      //
      setFlywheelPctVBus = (speed) -> 
      {
        flywheelMotor.set(TalonFXControlMode.PercentOutput, speed);
        check(flywheelMotor, "set %VBus error", false);
      };

      setFlywheelSpeed = (speed) -> 
      {
        flywheelMotor.set(TalonFXControlMode.Velocity, speed);
        check(flywheelMotor, "set speed error", false);
      };

      getFlywheelSpeed = () ->
      {
        var speed = flywheelMotor.getSelectedSensorVelocity(pidIdx);
        check(flywheelMotor, "get sensor error", false);
        return speed;
      };

      getSpeedError = () ->
      {
        var loopError = flywheelMotor.getClosedLoopError(pidIdx);
        check(flywheelMotor, "get velocity_error error", false);
        return loopError;
      };

      getPctOutput = () ->
      {
        var pctOutput = flywheelMotor.getMotorOutputPercent();
        check(flywheelMotor, "get %VBus error", false);
        return pctOutput;
      };

      getBusVoltage = () ->
      {
        var volts = flywheelMotor.getBusVoltage();
        check(flywheelMotor, "get voltage error", false);
        return volts;
      };

      isVoltageCompensationEnabled = () ->
      {
        var enabled = flywheelMotor.isVoltageCompensationEnabled();
        check(flywheelMotor, "auto check comp error", false);
        return enabled;
      };
      
      // method to display stuff
      printSpeed = () ->
      {
        SmartDashboard.putNumber("velocity measured (native units)", getFlywheelSpeed.get());
        SmartDashboard.putNumber("velocity measured (RPM)", getFlywheelSpeed.get() * nativeToRPM);
        SmartDashboard.putNumber("error (native units)", getSpeedError.get());
        SmartDashboard.putNumber("error (RPM)", getSpeedError.get() * nativeToRPM);
        SmartDashboard.putNumber("kF tentative", PctVBusToThrottle *
            (isVoltageCompensationEnabled.get() ? getBusVoltage.get() / voltageCompensation : 1.) * // voltage compensation correction factor
             getPctOutput.get() / getFlywheelSpeed.get() );
        SmartDashboard.putNumber("%VBus", getPctOutput.get());
        SmartDashboard.putNumber("bus voltage", getBusVoltage.get());
        SmartDashboard.updateValues();
      };

      return errors == 0;
    }

  /** Check the TalonFX function for an error and print a message
   * 
   * @param TalonFX
   * @param message to print
   * @param printAll flag to print all (true) or just errors (false)
   * @return 1 for error and 0 for no error
   */
  public static int check(TalonFX motorController, String message, boolean printAll)
  {  
    var rc = motorController.getLastError();
    if(rc != ErrorCode.OK || printAll)
    {
      System.out.println("[Talon] " + message + " " + rc);
    }
    return rc == ErrorCode.OK ? 0 : 1;
  }

}
/*
********** Robot program starting **********
[Talon] clear faults OK
[Talon] set default OK
[Talon] set status 2 OK
[Talon] set status 13 OK
[Talon] set inverted OK
[Talon] set neutral mode OK
[Talon] set sensor position OK
NT: server: client CONNECTED: 10.42.37.5 port 53261
[Talon] set configs OK
CTR: No new response to update signal
CTR: No new response to update signal
[Talon] get configs OK
[Talon] configuration:
.supplyCurrLimit = Limiting is disabled.;
.statorCurrLimit = Limiting is disabled.;
.motorCommutation = Trapezoidal;
.absoluteSensorRange = Unsigned: 0 to 360 deg (positive full rotation;
.integratedSensorOffsetDegrees = 0.0;
.initializationStrategy = On boot up, set position to zero.;
.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor;
.primaryPID.selectedFeedbackCoefficient = 1.0;
.auxiliaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor;
.auxiliaryPID.selectedFeedbackCoefficient = 1.0;
.forwardLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
.reverseLimitSwitchSource = LimitSwitchSource.FeedbackConnector;
.forwardLimitSwitchDeviceID = 0;
.reverseLimitSwitchDeviceID = 0;
.forwardLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
.reverseLimitSwitchNormal = LimitSwitchNormal.NormallyOpen;
.sum0Term = TalonFXFeedbackDevice.IntegratedSensor;
.sum1Term = TalonFXFeedbackDevice.IntegratedSensor;
.diff0Term = TalonFXFeedbackDevice.IntegratedSensor;
.diff1Term = TalonFXFeedbackDevice.IntegratedSensor;
.openloopRamp = 0.0;
.closedloopRamp = 0.0;
.peakOutputForward = 1.0;
.peakOutputReverse = 0.0;
.nominalOutputForward = 0.0;
.nominalOutputReverse = 0.0;
.neutralDeadband = 9.775171065493646E-4;
.voltageCompSaturation = 10.0;
.voltageMeasurementFilter = 32;
.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_5Ms;
.velocityMeasurementWindow = 1;
.forwardSoftLimitThreshold = 0.0;
.reverseSoftLimitThreshold = 0.0;
.forwardSoftLimitEnable = false;
.reverseSoftLimitEnable = false;
.slot0.kP = 0.09999990463256836;
.slot0.kI = 0.0;
.slot0.kD = 0.0;
.slot0.kF = 0.05549979209899902;
.slot0.integralZone = 0.0;
.slot0.allowableClosedloopError = 0.0;
.slot0.maxIntegralAccumulator = 0.0;
.slot0.closedLoopPeakOutput = 1.0;
.slot0.closedLoopPeriod = 1;
.slot1.kP = 0.0;
.slot1.kI = 0.0;
.slot1.kD = 0.0;
.slot1.kF = 0.0;
.slot1.integralZone = 0.0;
.slot1.allowableClosedloopError = 0.0;
.slot1.maxIntegralAccumulator = 0.0;
.slot1.closedLoopPeakOutput = 1.0;
.slot1.closedLoopPeriod = 1;
.slot2.kP = 0.0;
.slot2.kI = 0.0;
.slot2.kD = 0.0;
.slot2.kF = 0.0;
.slot2.integralZone = 0.0;
.slot2.allowableClosedloopError = 0.0;
.slot2.maxIntegralAccumulator = 0.0;
.slot2.closedLoopPeakOutput = 1.0;
.slot2.closedLoopPeriod = 1;
.slot3.kP = 0.0;
.slot3.kI = 0.0;
.slot3.kD = 0.0;
.slot3.kF = 0.0;
.slot3.integralZone = 0.0;
.slot3.allowableClosedloopError = 0.0;
.slot3.maxIntegralAccumulator = 0.0;
.slot3.closedLoopPeakOutput = 1.0;
.slot3.closedLoopPeriod = 1;
.auxPIDPolarity = false;
.filter0.remoteSensorDeviceID = 0;
.filter0.remoteSensorSource = RemoteSensorSource.Off;
.filter1.remoteSensorDeviceID = 0;
.filter1.remoteSensorSource = RemoteSensorSource.Off;
.motionCruiseVelocity = 0.0;
.motionAcceleration = 0.0;
.motionCurveStrength = 0;
.motionProfileTrajectoryPeriod = 0;
.feedbackNotContinuous = false;
.remoteSensorClosedLoopDisableNeutralOnLOS = false;
.clearPositionOnLimitF = false;
.clearPositionOnLimitR = false;
.clearPositionOnQuadIdx = false;
.limitSwitchDisableNeutralOnLOS = false;
.softLimitDisableNeutralOnLOS = false;
.pulseWidthPeriod_EdgesPerRot = 0;
.pulseWidthPeriod_FilterWindowSz = 0;
.trajectoryInterpolationEnable = true;
.customParam0 = 0;
.customParam1 = 0;
[Talon] set enable compensation error OK
[Talon] compensation true
[Talon] compensation error OK
[Talon] 0 errors from config methods
********** Robot program startup complete **********
Default disabledPeriodic() method... Override me!
Default robotPeriodic() method... Override me!
[phoenix] Library initialization is complete.
[phoenix-diagnostics] Server 1.9.0 (Jan 4 2022,20:28:13) running on port: 1250
Loop time of 0.02s overrun
Warning at edu.wpi.first.wpilibj.IterativeRobotBase.printLoopOverrunMessage(IterativeRobotBase.java:359): Loop time of 0.02s overrun
0.00, 0.00, 0.00, 12.15, kF= NaN
SmartDashboard.updateValues(): 0.000188s
robotPeriodic(): 0.000069s
LiveWindow.updateValues(): 0.000009s
Shuffleboard.update(): 0.000022s
autonomousPeriodic(): 0.256744s
Warning at edu.wpi.first.wpilibj.Tracer.lambda$printEpochs$0(Tracer.java:63): SmartDashboard.updateValues(): 0.000188s
robotPeriodic(): 0.000069s
LiveWindow.updateValues(): 0.000009s
Shuffleboard.update(): 0.000022s
autonomousPeriodic(): 0.256744s
0.10, 1866.80, 0.08, 12.40, kF= 0.0555
0.20, 3780.00, 0.17, 12.16, kF= 0.0550
0.30, 5548.80, 0.25, 12.34, kF= 0.0561
0.40, 7404.40, 0.33, 12.32, kF= 0.0562
0.50, 9224.00, 0.41, 12.30, kF= 0.0563
0.60, 11068.00, 0.50, 12.30, kF= 0.0564
0.70, 12907.20, 0.58, 12.27, kF= 0.0564
0.80, 14748.40, 0.66, 12.26, kF= 0.0565
0.90, 16597.60, 0.75, 12.23, kF= 0.0565
1.00, 18446.00, 0.83, 12.19, kF= 0.0564
*/
