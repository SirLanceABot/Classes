/** Sample PIDF controller in one TalonFX motor controller
 * 
 * All the PIDF constants and filter times are built into the code.
 * The intention is they can be changed in the Phoenix Tuner
 * (or rewrite a bunch of code here)
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
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {

  TalonFX flywheelMotor;
  private static final int TIMEOUT_MS = 30; // milliseconds TalonFX command timeout limit
  Runnable printSpeed;
  Consumer<Double> setFlywheelSpeed;
  Supplier<Double> getFlywheelSpeed;
  Supplier<Double> getSpeedError;
  
  double speed = 0.; //initial speed to display on SmartDashboard
  
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
    final int filterWindow = 1; // ms
    final SensorVelocityMeasPeriod filterPeriod = SensorVelocityMeasPeriod.Period_5Ms;
    final int sampleTime = 10; // ms
    final double kP = 0.1;
    final double kI = 0.;
    final double kD = 0.;
    final double kF = 0.044;

    configFlywheel(flywheelMotorPort, pidIdx, filterWindow, filterPeriod, sampleTime, kP, kI, kD, kF);
  }

  @Override
  public void teleopPeriodic() {   
    speed = SmartDashboard.getNumber("velocity set (native units)", speed);
    setFlywheelSpeed.accept(speed);
    printSpeed.run();
  }

  /** Configure the motor controller
   * 
   * @param flywheelMotorPort CAN
   * @param pidIdx PID index 0 is either 0 or none
   * @param filterWindow ms
   * @param filterPeriod ms
   * @param sampleTime ms
   * @param kP
   * @param kI
   * @param kD
   * @param kF
   */
  void configFlywheel(int flywheelMotorPort, int pidIdx, int filterWindow, SensorVelocityMeasPeriod filterPeriod, int sampleTime, double kP, double kI, double kD, double kF)
  {
      flywheelMotor = new TalonFX(flywheelMotorPort);

      System.out.println("[Talon] set factory default " + flywheelMotor.configFactoryDefault(TIMEOUT_MS));

      System.out.println("[Talon] clear faults " + flywheelMotor.clearStickyFaults(TIMEOUT_MS));
      //
      // get the directions right. This assumes one way and your might be the other way
      //
      System.out.println("[Talon] set nominal output reverse " + flywheelMotor.configNominalOutputReverse(0., TIMEOUT_MS));

      flywheelMotor.setInverted(false);
      System.out.println("[Talon] set inverted error " + flywheelMotor.getLastError());
      
      flywheelMotor.setSensorPhase(false);
      System.out.println("[Talon] set sensor phase error " + flywheelMotor.getLastError());
      //
      //
      flywheelMotor.setNeutralMode(NeutralMode.Coast);
      System.out.println("[Talon] set neutral mode error " + flywheelMotor.getLastError());

      System.out.println("[Talon] set vel period " + flywheelMotor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_5Ms, TIMEOUT_MS));
      System.out.println("[Talon] set vel window " + flywheelMotor.configVelocityMeasurementWindow(1, TIMEOUT_MS));

      FeedbackDevice sensor = FeedbackDevice.IntegratedSensor;
      System.out.println("[Talon] set feedback sensor " + sensor.toString() + " " + flywheelMotor.configSelectedFeedbackSensor(sensor, 0, TIMEOUT_MS));

      System.out.println("[Talon] set status 2 " + flywheelMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, sampleTime, TIMEOUT_MS));
      System.out.println("[Talon] set status 13 " + flywheelMotor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, sampleTime, TIMEOUT_MS)); // PID error

      flywheelMotor.setSelectedSensorPosition(0, pidIdx, TIMEOUT_MS); // start at 0 position just for fun; not needed to tune velocity
      System.out.println("[Talon] set sensor position error " + flywheelMotor.getLastError());

      System.out.println("[Talon] config meas window " + flywheelMotor.configVelocityMeasurementWindow(filterWindow, TIMEOUT_MS));
      System.out.println("[Talon] config meas period " + flywheelMotor.configVelocityMeasurementPeriod(filterPeriod, TIMEOUT_MS));

      System.out.println("[Talon] config kP " + flywheelMotor.config_kP(pidIdx, kP, TIMEOUT_MS));
      System.out.println("[Talon] config kI " + flywheelMotor.config_kI(pidIdx, kI, TIMEOUT_MS));
      System.out.println("[Talon] config kD " + flywheelMotor.config_kD(pidIdx, kD, TIMEOUT_MS));
      System.out.println("[Talon] config kF " + flywheelMotor.config_kF(pidIdx, kF, TIMEOUT_MS));

      // get and display the motor configuration
      TalonFXConfiguration allConfigs = new TalonFXConfiguration();
      System.out.println("[Talon] get config error " + flywheelMotor.getLastError());
      System.out.println("[Talon] flywheel motor configs\n" + allConfigs);
      
      //
      // methods to access the TalonFX motor controller
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
        if(flywheelMotor.getLastError() != ErrorCode.OK) System.out.println("get error error " + flywheelMotor.getLastError());
        return error;
      };

      // method to display stuff
      printSpeed = () ->
      {
        var nativeToRPM = 10. * 60. / 2048.; // 10 .1sec/sec   60 secs/min    2048 encoder ticks/rev
        SmartDashboard.putNumber("velocity measured (native units)", getFlywheelSpeed.get());
        SmartDashboard.putNumber("velocity measured (RPM)", getFlywheelSpeed.get() * nativeToRPM);
        SmartDashboard.putNumber("error (native units)", getSpeedError.get());
        SmartDashboard.putNumber("error (RPM)", getSpeedError.get() * nativeToRPM);
      };
    }
}
