package org.usfirst.frc.team4237.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import org.usfirst.frc.team4237.robot.camera_process.targetinfo;
/**
 * This is a demo program showing the use of OpenCV to do vision processing. The
 * image is acquired from the USB camera then
 * sent to the dashboard. OpenCV has many methods for different types of
 * processing.
 */
public class Robot extends IterativeRobot {
	Thread visionThread;
	public camera_process cp;

	@Override
	public void robotInit() {
		System.out.println("[robotInit] starting");
		cp = new camera_process();
		visionThread = new Thread(cp);
		visionThread.setDaemon(true); // defines a sort of "background" task that just keeps running (until all the normal threads have terminated; must set before the ".start"
		visionThread.start();
	}

	public void robotPeriodic()
	{
		// debug output
		// do not fetch new data here if it is fetched in other methods such as autonomous otherwise the data may never appear as fresh
		targetinfo ti = cp.new targetinfo(); // get the latest target information
												// data must be fetched as new from only one method as that resets the master Fresh/Stale flag
												// The isFresh() method indicates if this new data actually came form the same camera image (frame) as the previous new data
		
		if(ti.isFresh()) // fresh means at the time it was fetched as new it had not been previously seen
							// a new camera frame has been processed a received here (although the current image might be identical to the previous image if the scene hasn't changed enough)
							// it is conceivable that depending on the frame rate and timing of interleaved processes that a new camera frame became available between the time of the new data constructor and this call to isFresh()
			System.out.println("[robotPeriodic] " + ti + " Fresh ");
		else
			System.out.println("[robotPeriodic] " + ti + " Stale ");
	}

	@Override
	public void autonomousInit()
	{
		System.out.println("[autonomousInit] starting so Autonomous vision thread needed so tell it to awaken");
		// wake up the camera thread if it happens to be sleeping
		synchronized(Constants.LOCKS.cameraThread)
		{
			Constants.LOCKS.cameraThread.notify();
		}
	}
	
	@Override
	public void teleopInit()
	{
		System.out.println("[teleopInit] starting so Autonomous vision thread not needed so tell it to sleep");
		visionThread.interrupt();
	}

	@Override	public void autonomousPeriodic() {System.out.println("[autonomousPeriodic]");}
	@Override	public void teleopPeriodic(){System.out.println("[teleopPeriodic]");}
	@Override	public void testInit() {System.out.println("[testInit]");}
	@Override	public void testPeriodic() {System.out.println("[testPeriodic]");}
	@Override	public void disabledInit() {System.out.println("[disabledInit]");}
	@Override	public void disabledPeriodic() {System.out.println("[disabledPeriodic]");}
}
