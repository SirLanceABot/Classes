package org.usfirst.frc.team4237.robot;

import edu.wpi.first.wpilibj.IterativeRobot;

import static org.usfirst.frc.team4237.robot.Constants.GYRO;

public class Robot extends IterativeRobot {

	private ITG3200 mGyro;

	@Override
	public void robotInit()
	{
		mGyro = new ITG3200(GYRO.SIX_DOF_PORT, GYRO.ADDRESS, GYRO.SAMPLE_RATE_FILTER, GYRO.INITIAL_ANGLE, GYRO.GYRO_SENSITIVITY);
	}

	@Override
	public void teleopInit()
	{
		System.out.println("Selected Gyro Test");
	}

	@Override
	public void teleopPeriodic()
	{
		System.out.println(mGyro);
	}
}
