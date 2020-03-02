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
		{
			if(mLIDAR.IsDistanceAvailable())
			{
				System.out.println("good " + mLIDAR);// do stuff - it's good
			}
			else
			{
				System.out.println("bad " + mLIDAR);// do stuff - last distance scan wasn't right (returns -1) so try it again
			}
		}
		else
		{
			System.out.println("LIDAR-LITE DOA");// do stuff for dead LIDAR there is no hope
		} 
	}
}
