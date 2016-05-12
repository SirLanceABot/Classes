#include "WPILib.h"

/**
 * This is a demo program showing the use of the RobotDrive class.
 * The SampleRobot class is the base of a robot application that will automatically call your
 * Autonomous and OperatorControl methods at the right time as controlled by the switches on
 * the driver station or the field controls.
 *
 * WARNING: While it may look like a good choice to use for your code if you're inexperienced,
 * don't. Unless you know what you are doing, complex code will be much more difficult under
 * this system. Use IterativeRobot or Command-Based instead if you're new.
 */
class Robot: public SampleRobot
{
	//RobotDrive* myRobot; // robot drive system
	Talon* mMotor;
	Joystick* Xbox; // only joystick
	PowerDistributionPanel* PDP;
	BuiltInAccelerometer* PullingGs;
	DigitalInput* mSwitch;

public:
	Robot()
	{
		printf("Starting Robot Constructor\n");
		//myRobot = new RobotDrive(0,1);
		Xbox = new Joystick(0);
		PDP = new PowerDistributionPanel();
		PullingGs = new BuiltInAccelerometer;

		//myRobot->SetExpiration(0.1);
		mMotor = new Talon(1);
		mSwitch = new DigitalInput(9);
	}

	/**
	 * Runs the motors with arcade steering.
	 */
	void OperatorControl()
	{
		double temp = -99;
		printf("Starting OperatorControl\n");
		//fflush(NULL);
		PDP->ClearStickyFaults();
		printf("PDP Sticky faults cleared\n");
		//fflush(NULL);
		temp = PDP->GetTemperature();
		printf("%f %f %f %f %f\n",
			PDP->GetTemperature(), PDP->GetTotalCurrent(), PDP->GetTotalEnergy(), PDP->GetTotalPower(), PDP->GetVoltage());
		//fflush(NULL);
		//myRobot->SetSafetyEnabled(false);

		while (IsOperatorControl()/* && IsEnabled()*/)
		{
			printf("Running OperatorControl\n");
			//fflush(NULL);
			printf("x y z Gs %f %f %f\n", PullingGs->GetX(), PullingGs->GetY(), PullingGs->GetZ() );
			//fflush(NULL);
			printf("%f %f %f %f %f\n",
				PDP->GetCurrent(15), PDP->GetTotalCurrent(), PDP->GetTotalEnergy(), PDP->GetTotalPower(), PDP->GetVoltage());
			//fflush(NULL);

			if(Xbox->GetRawButton(1) == true)
				mMotor->Set(0.5);
			else if(mSwitch->Get() == true)
				mMotor->Set(1.0);
			else if(GetUserButton() == true)
				mMotor->Set(-0.5);
			else
				mMotor->Disable();

			//myRobot->TankDrive(Xbox->GetRawAxis(2), Xbox->GetRawAxis(4), true);
			Wait(0.01);				// wait for a motor update time
		}
	}



};

START_ROBOT_CLASS(Robot);
