package org.usfirst.frc.team4237.robot;

import edu.wpi.first.networktables.NetworkTableType;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.IterativeRobot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends IterativeRobot {

	myPIDController_Custom_1 m_myPIDController_Custom_1 = new myPIDController_Custom_1(); //TODO set all the parameters for the process and controller

	PlotThread _plotThread = null;
	StringBuilder _sb = new StringBuilder();
	int _loops;
	double Kp_prev, Ki_prev, Kd_prev, Kf_prev;

	@Override
	public void robotInit() {
		_loops = 500;
		_sb.setLength(0);
	}

	@Override
	public void teleopInit() {
		System.out.println(System.currentTimeMillis() + " periodic start time ms");
		SmartDashboard.putBoolean("Use ClosedLoopVelocity", false);
		SmartDashboard.putNumber("Enter Kp", 0.67 );
		SmartDashboard.putNumber("Enter Ki", 0.06);
		SmartDashboard.putNumber("Enter Kd", 0.06);
		SmartDashboard.putNumber("Enter Kf", 200.);

		m_myPIDController_Custom_1.mPID.setP(0.67);
		m_myPIDController_Custom_1.mPID.setI(0.06);
		m_myPIDController_Custom_1.mPID.setD(0.06);
		m_myPIDController_Custom_1.mPID.setF(200.); // should be near what is needed for the steady state speed; they are related
		
		Kp_prev=0.67; Ki_prev=0.06; Kd_prev=0.06; Kf_prev=200.;

		System.out.println(m_myPIDController_Custom_1.mPID.getP());
		System.out.println(m_myPIDController_Custom_1.mPID.getI());
		System.out.println(m_myPIDController_Custom_1.mPID.getD());
		System.out.println(m_myPIDController_Custom_1.mPID.getF());
		System.out.println(m_myPIDController_Custom_1.mPID.getSetpoint());

		/* fire the plotter */
		_plotThread = new PlotThread(this);
		new Thread(_plotThread).start();

		m_myPIDController_Custom_1.mPID.enable();
	}

	/**
	 * This function is called periodically during operator control
	 * 
	 */
	@Override
	public void teleopPeriodic() {	
		double Kp, Ki, Kd, Kf;
		Kp = SmartDashboard.getNumber("Enter Kp", .67 );
		Ki = SmartDashboard.getNumber("Enter Ki", 0.06);
		Kd = SmartDashboard.getNumber("Enter Kd", 0.06);
		Kf = SmartDashboard.getNumber("Enter Kf", 200.);

		if(Kp_prev != Kp )
		{
			m_myPIDController_Custom_1.mPID.setP(Kp);
			Kp_prev = Kp;
			m_myPIDController_Custom_1.mPID.reset();
			m_myPIDController_Custom_1.mPID.enable();
		}

		if(Ki_prev != Ki )
		{
			m_myPIDController_Custom_1.mPID.setP(Ki);
			Ki_prev = Ki;
			m_myPIDController_Custom_1.mPID.reset();
			m_myPIDController_Custom_1.mPID.enable();
		}

		if(Kd_prev != Kd )
		{
			m_myPIDController_Custom_1.mPID.setP(Kd);
			Kd_prev = Kd;
			m_myPIDController_Custom_1.mPID.reset();
			m_myPIDController_Custom_1.mPID.enable();
		}

		if(Kf_prev != Kf )
		{
			m_myPIDController_Custom_1.mPID.setP(Kf);
			Kf_prev = Kf;
			m_myPIDController_Custom_1.mPID.reset();
			m_myPIDController_Custom_1.mPID.enable();
		}

//		if (m_myPIDController_Custom_1.mPID.onTarget())
//		{
//			System.out.println("At setpoint - turning off controller - done with simulation");
//			m_myPIDController_Custom_1.mPID.reset(); // pidGet continues after one call to pidWrite with 0.0 control signal
//		}
	}

	@Override
	public void disabledInit()
	{
		System.out.println("Disabled");
		// disabledInit is called before teleop or auto so there is 1 call each to pidGet and pidWrite to start with
		// if not stopped in teleop or auto then it keeps going unless stopped with reset here
		m_myPIDController_Custom_1.mPID.reset(); // pidWrite is called with 0.0 control signal one or two last times; then it's all stopped
		// but don't know about the thread still running - check the code
	}

	/** quick and dirty threaded plotter */
	class PlotThread implements Runnable {
		Robot robot;
		Boolean flipIncrement = false;
		public PlotThread(Robot robot) { this.robot = robot; }

		public void run() {
			System.out.print("In run of plot thread");
			/* speed up network tables, this is a test project so eat up all 
			 * of the network possible for the purpose of this test.
			 */
			//NetworkTable..setUpdateRate(0.020); /* this suggests each time unit is 20ms in the plot */
			while (true) {
				/* yield for a ms or so - this is not meant to be accurate */
				try { Thread.sleep(19); } catch (Exception e) { }
				/* grab the last signal update from our frame update */
				double speed = m_myPIDController_Custom_1.processVariable;
				double speedRequested = m_myPIDController_Custom_1.mPID.getSetpoint();
				SmartDashboard.putNumber("speed requested", speedRequested);
				SmartDashboard.putNumber("speed", speed);
				if (flipIncrement) // fake out to keep plot going
				{
					speed += Math.ulp(speed);
					speedRequested += Math.ulp(speedRequested);
					flipIncrement = false;
				}
				else flipIncrement = true;
				SmartDashboard.putNumber("spd_request", speedRequested);
				SmartDashboard.putNumber("spd", speed);
			}
		}
	}
}
