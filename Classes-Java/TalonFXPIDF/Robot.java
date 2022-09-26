/** Sample PIDF controller in one CAN TalonFX motor controller using Integrated Sensor
 * 
 * All the PIDF constants and filter times are built into the code and are okay for no load.
 * The intention is they can be changed in the Phoenix Tuner to tune a real device.
 * 
 * The setpoint speed is entered on the SmartDashboard as "velocity set (native units)"
 * It is the only input to the program.
 */

package frc.robot;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  TalonFX flywheelMotor;
  private static final int TIMEOUT_MS = 30; // milliseconds TalonFX command timeout limit
  Runnable printSpeed;
  Consumer<Double> setFlywheelSpeed;
  Supplier<Double> getFlywheelSpeed;
  Supplier<Double> getSpeedError;
  
  double speed = 0.; //initial speed to run and display on SmartDashboard
  // It seemed that sometimes a previous value from the SmartDashboard is used.
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
    final int pidIdx = 0; // Talon primary closed loop control (or none)
    final boolean invert = false;
    final int filterWindow = 1; // ms
    final SensorVelocityMeasPeriod filterPeriod = SensorVelocityMeasPeriod.Period_5Ms;
    final int sampleTime = 10; // ms
    final double kP = 0.1;
    final double kI = 0.;
    final double kD = 0.;
    final double kF = 0.044;

    configFlywheel(flywheelMotorPort, pidIdx, invert, filterWindow, filterPeriod, sampleTime, kP, kI, kD, kF);

    Timer.delay(0.2); // let everything settle - Phoenix starting and SmartDashboard updating
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
  void configFlywheel(int flywheelMotorPort, int pidIdx,  boolean invert,
                      int filterWindow, SensorVelocityMeasPeriod filterPeriod, int sampleTime,
                      double kP, double kI, double kD, double kF)
  {
      flywheelMotor = new TalonFX(flywheelMotorPort);

      System.out.println("[Talon] clear faults " + flywheelMotor.clearStickyFaults(TIMEOUT_MS));

      System.out.println("[Talon] set status 2 " + flywheelMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, sampleTime, TIMEOUT_MS));
      System.out.println("[Talon] set status 13 " + flywheelMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, sampleTime, TIMEOUT_MS)); // PID error

      flywheelMotor.setInverted(invert);
      System.out.println("[Talon] set inverted " + flywheelMotor.getLastError());

      flywheelMotor.setNeutralMode(NeutralMode.Coast);
      System.out.println("[Talon] set neutral mode " + flywheelMotor.getLastError());

      flywheelMotor.setSelectedSensorPosition(0, pidIdx, TIMEOUT_MS); // start at 0 position just for fun; not needed to tune velocity
      System.out.println("[Talon] set sensor position " + flywheelMotor.getLastError());
     
      // get the factory defaults for some setting as defined by this Java API - may be different than Phoenix Tuner
			TalonFXConfiguration configs = new TalonFXConfiguration();
      // change the ones that we need to

      // configs.voltageCompSaturation = 11.; // potentially useful to limit somewhat and enable tuning at realistic voltage and for reproducibility but not now
      // flywheelMotor.enableVoltageCompensation(true);

      configs.nominalOutputReverse = invert ? -1. : 0.;
      configs.primaryPID.selectedFeedbackSensor = FeedbackDevice.IntegratedSensor; // TalonFXFeedbackDevice doesn't work for some reason
      configs.velocityMeasurementWindow = filterWindow;
      configs.velocityMeasurementPeriod = filterPeriod;
      configs.slot0.kP = kP;
      configs.slot0.kI = kI;
      configs.slot0.kD = kD;
      configs.slot0.kF = kF;

			System.out.println("[Talon] set configs " + flywheelMotor.configAllSettings(configs)); // send the new config back

      System.out.println("[Talon] get configs " + flywheelMotor.getAllConfigs(configs, TIMEOUT_MS)); // read them back
      System.out.println("[Talon] configuration:\n" + configs); // print them

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
        var nativeToRPM = 10. * 60. / 2048.; // 10 .1sec/sec   60 secs/min    2048 encoder ticks/rev for Integrated Sensor
        SmartDashboard.putNumber("velocity measured (native units)", getFlywheelSpeed.get());
        SmartDashboard.putNumber("velocity measured (RPM)", getFlywheelSpeed.get() * nativeToRPM);
        SmartDashboard.putNumber("error (native units)", getSpeedError.get());
        SmartDashboard.putNumber("error (RPM)", getSpeedError.get() * nativeToRPM);
        SmartDashboard.putNumber("kF tentative", 1023.*flywheelMotor.getMotorOutputPercent()/getFlywheelSpeed.get());
        SmartDashboard.putNumber("bus voltage", flywheelMotor.getBusVoltage());
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

