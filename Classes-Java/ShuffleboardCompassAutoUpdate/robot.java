/*
 Minimal example of using a Shuffleboard complex GYRO widget for our own
 calculated "gyro" and updating it automatically on every iteration.

 The SYSTEM automatically calls a class getter every iteration for anything
 added to the SendableRegistry.

 For the GYRO widget it calls getAngle.

 The example angle returned is the iteration count divided by 100 so the
 shuffleboard compass slowly rotates clockwise.
*/

package frc.robot;

import java.util.Map;

import edu.wpi.first.wpilibj.GyroBase;
import edu.wpi.first.wpilibj.Sendable;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableRegistry;

public class Robot extends TimedRobot {
  Gyro gyro;

  @Override
  public void robotInit()
  {
    gyro = new myGyro();
  }

  static class myGyro extends GyroBase implements Gyro, Sendable, AutoCloseable 
    {
        static double count = 0.; // example angle value

        myGyro()
        {
            Shuffleboard.getTab("Camera")
            .add("Turret Field Angle", (Sendable) this)
            .withWidget(BuiltInWidgets.kGyro)
            .withPosition(22, 7).withSize(7, 7)
            .withProperties(Map.of(
              "Major tick spacing", 30.,     // default 45
              "Starting angle	Number", 0.,   // default 180
              "Show tick mark ring", true))  // default true
            ;
        
          Shuffleboard.update();

          SendableRegistry.add(this, "Turret Field Angle");
        }

        @Override
        public double getAngle()
        {
          return count++/100.; // example angle value updated and returned to requester
        }

        @Override
        public void reset(){}

        @Override
        public double getRate()
        {
          return 0.;
        }

        @Override
        public void calibrate(){}

        @Override
        public void close()
        {
          SendableRegistry.remove(this);
        }
    }
}
