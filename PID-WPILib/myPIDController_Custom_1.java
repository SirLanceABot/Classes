package org.usfirst.frc.team4237.robot;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;

//TODO for each WPILIB based PID controller copy this entire class and customize for the device being controlled
public class myPIDController_Custom_1
{
	public double processVariable;
	public double controlSignal;
	public PIDController mPID;

	public class myPIDSource implements PIDSource
	{
		PIDSourceType PIDSourceType;

		@Override
		public void setPIDSourceType(PIDSourceType pidSourceType) {
			PIDSourceType = pidSourceType;
		}

		@Override
		public PIDSourceType getPIDSourceType() {
			return PIDSourceType;
		}

		@Override
		public double pidGet() {
			//TODO get the process variable from someplace and return it to the PID controller
			// this simulation sets the processVariable elsewhere so it's ready to use here at any time
			System.out.println(System.currentTimeMillis() + " pidGet called and returned PV " + processVariable);
			return processVariable;  //  where the process is now
		}
	}

	public class myPIDOutput implements PIDOutput
	{
		@Override
		public void pidWrite(double output) {
			//TODO PID controller puts the new control signal here so tell device to use it

			// for this simulation compute a new process variable value as a response from this control signal
			// and save it for the next controller input
			controlSignal = output;
			// this simulation is really simple: the process responds to the control signal by adding 30 to it
			processVariable = controlSignal + 30; // simulated response to the control signal
			System.out.println(System.currentTimeMillis() + " pidWrite called with control signal " + controlSignal +
					" and process responded with new PV " + processVariable + " " + mPID.getError());
		}
	}

	public class myPIDController extends PIDController
	{
		@Override
		protected double calculateFeedForward() {
			System.out.println("feed forward called");
			return mPID.getF();
		}

		myPIDController(double Kp, double Ki, double Kd,  double Kf, PIDSource processVariable_source, PIDOutput controlSignal_sink, double period)
		{
			super(Kp, Ki, Kd,  Kf, processVariable_source, controlSignal_sink, period);
		}
	}

	public myPIDController_Custom_1()
	{
		//TODO all the parameters of the PID controller and the process

		/////////////////////////////////////////////////////////////////
		// process variable characteristics
		double minimum_processVariable = 0;
		double maximum_processVariable = 500;
		processVariable = 0;
		double processVariableSetpoint = 270.;
		double tolerance_processVariable = 0.3;
		PIDSourceType processVariableType = PIDSourceType.kDisplacement;

		// control signal characteristics
		double minimum_controlSignal = -500;
		double maximum_controlSignal = 500;
		controlSignal = 0;

		// PID controller characteristics
		double Kp = .67;
		double Ki = 0.06;
		double Kd = 0.06;
		double Kf = 200;

		double period = .050;  // controller loop period
		// pidGet is called about every 20 milliseconds and just before pidWrite which is called every 50 milliseconds

		// end of all parameters of the PID controller and process
		////////////////////////////////////////////////////////////////

		myPIDSource processVariable_source = new myPIDSource();
		processVariable_source.setPIDSourceType(processVariableType);

		myPIDOutput controlSignal_sink = new myPIDOutput();
		mPID = new myPIDController(Kp, Ki, Kd,  Kf, processVariable_source, controlSignal_sink, period);
		mPID.setInputRange(minimum_processVariable, maximum_processVariable);
		mPID.setSetpoint(processVariableSetpoint);
		mPID.setOutputRange(minimum_controlSignal, maximum_controlSignal);
		mPID.setAbsoluteTolerance(tolerance_processVariable);
	}
}

///**
//* Calculate the feed forward term.
//*
//* <p>Both of the provided feed forward calculations are velocity feed forwards. If a different
//* feed forward calculation is desired, the user can override this function and provide his or
//* her own. This function  does no synchronization because the PIDController class only calls it
//* in synchronized code, so be careful if calling it oneself.
//*
//* <p>If a velocity PID controller is being used, the F term should be set to 1 over the maximum
//* setpoint for the output. If a position PID controller is being used, the F term should be set
//* to 1 over the maximum speed for the output measured in setpoint units per this controller's
//* update period (see the default period in this class's constructor).
//*/
////protected double calculateFeedForward() {
////if (m_pidInput.getPIDSourceType().equals(PIDSourceType.kRate)) {
//// return m_F * getSetpoint();
////} else {
//// double temp = m_F * getDeltaSetpoint();
//// m_prevSetpoint = m_setpoint;
//// m_setpointTimer.reset();
//// return temp;
////}
////}
}
