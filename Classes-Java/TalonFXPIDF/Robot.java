/** Sample PIDF controller in one CAN TalonFX motor controller using Integrated Sensor
 * 
 * All the PIDF constants and filter times are built into the code and are okay for no load
 * on a junk (rebuilt but damaged) motor.
 * The intention is they can be changed in the Phoenix Tuner to tune a real device.
 * 
 * The setpoint speed is entered on the SmartDashboard as "velocity set (native units)"
 * It is the only input to the program.
 * Press TAB to have the input value sent (ENTER works but then the robot is disabled)
 */

package frc.robot;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.ControlMode;
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

public class Robot extends TimedRobot {

  TalonFX flywheelMotor;
  private static final int TIMEOUT_MS = 50; // milliseconds TalonFX command timeout limit
  Runnable printSpeed;
  Consumer<Double> setFlywheelSpeed;
  Supplier<Double> getFlywheelSpeed;
  Supplier<Double> getSpeedError;
  
  double speed = 0.; //initial speed to run and display on SmartDashboard
  // It seemed that sometimes a previous value from the SmartDashboard is used (race condition?).
  // This code tries hard to prevent but not sure it's perfect or what the issue was.
  
  Robot()
  {
    LiveWindow.disableAllTelemetry(); // don't waste time on stuff we don't need
  }

  @Override
  public void robotInit() {
    
    SmartDashboard.putNumber("velocity set (native units)", speed);
    SmartDashboard.updateValues();

    final int flywheelMotorPort = 0;
    final double voltageCompensation = 10.;
    final int pidIdx = 0; // Talon primary closed loop control (or none)
    final boolean invert = false;
    final int filterWindow = 1; // ms
    final SensorVelocityMeasPeriod filterPeriod = SensorVelocityMeasPeriod.Period_5Ms; // 10ms and 20ms had less fluctuation
    final int sampleTime = 15; // ms
    final double kP = 0.16; // good around 8000 nu, 0.1 works for the entire velocity range
    final double kI = 0.; // no effect then suddenly bad
    final double kD = 0.; // no effect then suddenly bad
    final double kF = 0.046; // very good around 8000 nu and okay for the entire velocity range with a little more error creeping in

    configFlywheel(flywheelMotorPort, voltageCompensation, pidIdx, invert, filterWindow, filterPeriod, sampleTime, kP, kI, kD, kF);

    Timer.delay(0.2); // let everything settle - Phoenix starting and SmartDashboard updating
  }

  // calculate kF variables
  int count;
  double kfSpeed;
  double averageSpeed;
  double averagePctVoltage;

  @Override
  public void autonomousInit()
  {
    count = 0;
    kfSpeed = 0.;
    averageSpeed = 0.;
    averagePctVoltage = 0.;
  }

  /**
   * print values related to kF calculation
   * collect data at 10 %VBus settings from plus the 0 point
   * skip 50 iterations to let speed settle
   * average values from the next 50 iterations
   * Set %VBus differs from the %VBus retrieved from the TalonFX for unknown reasons
   * 
   */
  @Override
  public void autonomousPeriodic()
  {
    if(kfSpeed > 1.01)
    {
      flywheelMotor.set(ControlMode.PercentOutput, 0.);
      return;
    }

    count++;

    flywheelMotor.set(ControlMode.PercentOutput, kfSpeed);

    if(count <= 50) return; // skip 50 iterations
    // then gather data for 50
    averageSpeed += getFlywheelSpeed.get();
    averagePctVoltage += flywheelMotor.getMotorOutputPercent();

    if(count < 100) return;
    // at iteration 100 print the smooth data and step up to next %VBus
    averageSpeed /= 50.;
    averagePctVoltage /= 50.;
    System.out.format("%5.2f %5.2f %5.2f %10.7f %7.4f\n",
          kfSpeed, averageSpeed, averagePctVoltage, averagePctVoltage/averageSpeed, 1023.*averagePctVoltage/averageSpeed);
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
    Timer.delay(0.2);
  }

  @Override
  public void teleopPeriodic() {   
    speed = SmartDashboard.getNumber("velocity set (native units)", speed);
    setFlywheelSpeed.accept(speed);
    printSpeed.run();
  }

  /** Configure the Talon motor controller
   * 
   * @param flywheelMotorPort CAN
   * @param voltageCompensation limit motor input - 0. disables voltage compensation
   * @param pidIdx PID index 0 is either 0 or none
   * @param invert (motor reversed)
   * @param filterWindow ms
   * @param filterPeriod ms
   * @param sampleTime ms
   * @param kP
   * @param kI
   * @param kD
   * @param kF
   */
  void configFlywheel(int flywheelMotorPort, double voltageCompensation, int pidIdx,  boolean invert,
                      int filterWindow, SensorVelocityMeasPeriod filterPeriod, int sampleTime,
                      double kP, double kI, double kD, double kF)
  {
      flywheelMotor = new TalonFX(flywheelMotorPort);

      System.out.println("[Talon] clear faults " + flywheelMotor.clearStickyFaults(TIMEOUT_MS));

      System.out.println("[Talon] set default " + flywheelMotor.configFactoryDefault(TIMEOUT_MS));
      
      System.out.println("[Talon] set status 2 " + flywheelMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, sampleTime, TIMEOUT_MS));
      System.out.println("[Talon] set status 13 " + flywheelMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, sampleTime, TIMEOUT_MS)); // PID error
      //System.out.println("[Talon] set status 10 " + flywheelMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, sampleTime, TIMEOUT_MS)); // this may or may not be useful

      flywheelMotor.setInverted(invert);
      System.out.println("[Talon] set inverted " + flywheelMotor.getLastError());

      flywheelMotor.setNeutralMode(NeutralMode.Coast);
      System.out.println("[Talon] set neutral mode " + flywheelMotor.getLastError());

      flywheelMotor.setSelectedSensorPosition(0, pidIdx, TIMEOUT_MS); // start at 0 position just for fun; not needed to tune velocity
      System.out.println("[Talon] set sensor position " + flywheelMotor.getLastError());
     
      // get the factory defaults for some setting as defined by this Java API - may be different than Phoenix Tuner
			TalonFXConfiguration configs = new TalonFXConfiguration();

      // change the ones that we need to

      // voltage compensation seems potentially useful but not for sure to enable tuning at realistic voltage and for reproducibility
      // kF has to be increased by the amount of voltage reduction in the compensation. Check kP, too.
      configs.voltageCompSaturation = voltageCompensation;
      
      // one direction only assumed and forced - these work for both inverted or not
      configs.peakOutputReverse = 0.;
      configs.peakOutputForward = 1.;

      configs.primaryPID.selectedFeedbackSensor = TalonFXFeedbackDevice.IntegratedSensor.toFeedbackDevice();

      configs.velocityMeasurementWindow = filterWindow;
      configs.velocityMeasurementPeriod = filterPeriod;
      configs.slot0.kP = kP;
      configs.slot0.kI = kI;
      configs.slot0.kD = kD;
      configs.slot0.kF = kF;

			System.out.println("[Talon] set configs " + flywheelMotor.configAllSettings(configs, TIMEOUT_MS)); // send the new config back

      System.out.println("[Talon] get configs " + flywheelMotor.getAllConfigs(configs, TIMEOUT_MS)); // read them back
      System.out.println("[Talon] configuration:\n" + configs); // print them

      if(voltageCompensation != 0.)
      {
        flywheelMotor.enableVoltageCompensation(true);
        if(flywheelMotor.getLastError() != ErrorCode.OK) System.out.println("[Talon] set enable compensation error " + flywheelMotor.getLastError());
      }

      System.out.println("[Talon] compensation " + flywheelMotor.isVoltageCompensationEnabled());
      if(flywheelMotor.getLastError() != ErrorCode.OK) System.out.println("[Talon] compensation error " + flywheelMotor.getLastError());

      //
      // methods for others to access the TalonFX motor controller
      //
      setFlywheelSpeed = (speed) -> 
      {
        flywheelMotor.set(TalonFXControlMode.Velocity, speed);
        if(flywheelMotor.getLastError() != ErrorCode.OK) System.out.println("set speed error " + flywheelMotor.getLastError());
      };

      getFlywheelSpeed = () ->
      {
        var speed = flywheelMotor.getSelectedSensorVelocity(pidIdx);
        if(flywheelMotor.getLastError() != ErrorCode.OK) System.out.println("get sensor error " + flywheelMotor.getLastError());
        return speed;
      };

      getSpeedError = () ->
      {
        var error = flywheelMotor.getClosedLoopError(pidIdx);
        if(flywheelMotor.getLastError() != ErrorCode.OK) System.out.println("get velocity_error error " + flywheelMotor.getLastError());
        return error;
      };

      // method to display stuff
      printSpeed = () ->
      {
        var nativeToRPM = 10. * 60. / 2048.; // 10 .1sec/sec   60 secs/min   rev/2048 encoder ticks for Integrated Sensor
        SmartDashboard.putNumber("velocity measured (native units)", getFlywheelSpeed.get());
        SmartDashboard.putNumber("velocity measured (RPM)", getFlywheelSpeed.get() * nativeToRPM);
        SmartDashboard.putNumber("error (native units)", getSpeedError.get());
        SmartDashboard.putNumber("error (RPM)", getSpeedError.get() * nativeToRPM);
        SmartDashboard.putNumber("kF tentative", 1023. * flywheelMotor.getMotorOutputPercent() / getFlywheelSpeed.get()); // 1023 talon throttle unit / 100%VBus
        SmartDashboard.putNumber("%VBus", flywheelMotor.getMotorOutputPercent());
        SmartDashboard.putNumber("bus voltage", flywheelMotor.getBusVoltage());
        SmartDashboard.updateValues();
      };
    }
}
/*
Java TalonFX API default configuration:
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
.peakOutputReverse = -1.0;
.nominalOutputForward = 0.0;
.nominalOutputReverse = 0.0;
.neutralDeadband = 0.04;
.voltageCompSaturation = 0.0;
.voltageMeasurementFilter = 32;
.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
.velocityMeasurementWindow = 64;
.forwardSoftLimitThreshold = 0.0;
.reverseSoftLimitThreshold = 0.0;
.forwardSoftLimitEnable = false;
.reverseSoftLimitEnable = false;
.slot0.kP = 0.0;
.slot0.kI = 0.0;
.slot0.kD = 0.0;
.slot0.kF = 0.0;
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
.pulseWidthPeriod_EdgesPerRot = 1;
.pulseWidthPeriod_FilterWindowSz = 1;
.trajectoryInterpolationEnable = true;
.customParam0 = 0;
.customParam1 = 0;
*/
/*
0.00 0.00 0.00 NaN NaN
0.10 1507.20 0.06 0.0000415 0.0425
0.20 3956.80 0.17 0.0000420 0.0430
0.30 6374.40 0.27 0.0000423 0.0433
0.40 8826.00 0.38 0.0000425 0.0435
0.50 11197.20 0.48 0.0000428 0.0438
0.60 13615.20 0.58 0.0000428 0.0438
0.70 15993.60 0.69 0.0000430 0.0440
0.80 18366.40 0.79 0.0000431 0.0440
0.90 20771.20 0.90 0.0000431 0.0441
1.00 22871.20 1.00 0.0000437 0.0447
*/