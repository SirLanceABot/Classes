package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;

public class Robot extends TimedRobot {

	private LIDAR_Lite mLIDAR;

	@Override
	public void robotInit()
	{
		mLIDAR = new LIDAR_Lite(Constants.LIDAR.PORT, Constants.LIDAR.ADDRESS);
	}

	@Override
	public void teleopInit()
	{
		System.out.println("Selected LIDAR_Lite Test");
	}

	@Override
	public void teleopPeriodic()
	{
		if(mLIDAR.IsWorking()) 
		{System.out.println(mLIDAR);} // do stuff - it's good
		else{} // do stuff - it's dead
	}
}
