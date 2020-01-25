/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package org.usfirst.frc.team4237.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Random;

import edu.wpi.first.wpilibj.BuiltInAccelerometer;

/**
 * Uses the CameraServer class to automatically capture video from a USB webcam
 * and send it to the FRC dashboard without doing any vision processing. This
 * is the easiest way to get camera images to the dashboard. Just add this to
 * the robotInit() method in your program.
 */
public class Robot extends IterativeRobot {

	BuiltInAccelerometer acc = new BuiltInAccelerometer();
	double accXPrev, accYPrev, accZPrev;

	@Override
	public void robotInit() {
		CameraServer.getInstance().startAutomaticCapture();
		accXPrev = acc.getX(); // best guess is previous acceleration same as current
		accYPrev = acc.getY();
		accZPrev = acc.getZ();
	}

	@Override
	public void robotPeriodic()
	{
		double Vcc = RobotController.getVoltage5V(); // get actual voltage of the nominal 5v bus
		//System.out.printf("Vcc %5.2f volts\n", Vcc);

		double accX = acc.getX(); // get the accelerometer values
		double accY = acc.getY();
		double accZ = acc.getZ();

		double jerkX = accX - accXPrev; // jerk is the derivative of acceleration (we are using fixed time steps so don't bother with delta t)
		double jerkY = accY - accYPrev;
		double jerkZ = accZ - accZPrev;

		SmartDashboard.putNumber("jerkX", fuzz(jerkX)); // plot jerk
		SmartDashboard.putNumber("jerkY", fuzz(jerkY));
		SmartDashboard.putNumber("jerkZ", fuzz(jerkZ));

		if (Math.abs(jerkX) >= 3.f || Math.abs(jerkY) >= 3.f || Math.abs(jerkZ) >= 3.f)
		{
			DriverStation.reportError("Collision", false);
			System.out.printf("Jerk x, y, z %5.1f %5.1f %5.1f\n", jerkX, jerkY, jerkZ);
		}
		
		else if (Math.abs(jerkX) >= 1.f || Math.abs(jerkY) >= 1.f || Math.abs(jerkZ) >= 1.f)
		{
			DriverStation.reportWarning("Hard Touch", false);
			System.out.printf("Jerk x, y, z %5.1f %5.1f %5.1f\n", jerkX, jerkY, jerkZ);
		}
				
		accXPrev = accX; // setup for next time step
		accYPrev = accY;
		accZPrev = accZ;
	}

	// fuzz to keep the SmartDashboard plot going with constant data - it stops if the data don't change
	private static Random random = new Random();

	static double fuzz(double arg)
	{
		switch (random.nextInt(3)) // [0, number-1]
		{
		case 0:  return arg + Math.ulp(arg);
		case 1:  return arg - Math.ulp(arg);
		default: return arg;
		}
	}

}