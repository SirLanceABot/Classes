package org.usfirst.frc.team4237.robot;

import edu.wpi.first.wpilibj.IterativeRobot;

public class Robot extends IterativeRobot {

	myPIDController_Custom_1 m_myPIDController_Custom_1 = new myPIDController_Custom_1(); //TODO set all the parameters for the process and controller

	StringBuilder _sb = new StringBuilder();
	int _loops;

	@Override
	public void robotInit() {
		_loops = 500;
		_sb.setLength(0);
	}

	@Override
	public void teleopInit() {
		System.out.println(System.currentTimeMillis() + " periodic start time ms");
		System.out.println(m_myPIDController_Custom_1.mPID.getP());
		System.out.println(m_myPIDController_Custom_1.mPID.getI());
		System.out.println(m_myPIDController_Custom_1.mPID.getD());
		System.out.println(m_myPIDController_Custom_1.mPID.getF());
		System.out.println(m_myPIDController_Custom_1.mPID.getSetpoint());

		m_myPIDController_Custom_1.mPID.enable();
	}

	/**
	 * This function is called periodically during operator control
	 * 
	 */
	@Override
	public void teleopPeriodic() {	

		if (m_myPIDController_Custom_1.mPID.onTarget())
		{
			System.out.println("At setpoint - turning off controller - done with simulation");
			m_myPIDController_Custom_1.mPID.reset(); // pidGet continues after one call to pidWrite with 0.0 control signal
		}
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
}
