package org.usfirst.frc.team4237.robot;

import edu.wpi.first.wpilibj.IterativeRobot;

import static org.usfirst.frc.team4237.robot.Constants.*;

public class Robot extends IterativeRobot {

	private LIDAR_Lite mLIDAR;

	@Override
	public void robotInit()
	{
		mLIDAR = new LIDAR_Lite(LIDAR.PORT, LIDAR.ADDRESS);
	}

	@Override
	public void teleopInit()
	{
		System.out.println("Selected LIDAR_Lite Test");
	}

	@Override
	public void teleopPeriodic()
	{
		System.out.println(mLIDAR);
	}
}
