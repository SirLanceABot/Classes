// --- Macros ---
#include "WPILib.h"
#define ID std::cout << "Compiled: " << __FILE__ << " " << __DATE__ << " " << __TIME__ << " " << __FUNCTION__ << /*" " << this << " " << __PRETTY_FUNCTION__" " << __LINE__<<*/ std::endl;
#define DBMESSAGE(y) SmartDashboard::PutString("Preference Message ",y)
#define DBPSTR(x,y) SmartDashboard::PutString("DB/String "#x, y)
#define DBPLED(x,y) SmartDashboard::PutBoolean("DB/LED "#x,y)
#define DBPSLIDER(x,y) SmartDashboard::PutNumber("DB/Slider "#x,y)
#define DBPBUTTON(x,y) SmartDashboard::PutBoolean("DB/Button "#x,y)

#define DBGSTR(x) SmartDashboard::GetString("DB/String "#x)
#define DBGLED(x) SmartDashboard::GetBoolean("DB/LED "#x)
#define DBGSLIDER(x) SmartDashboard::GetNumber("DB/Slider "#x)
#define DBGBUTTON(x) SmartDashboard::GetBoolean("DB/Button "#x)

#include "WPILib.h"

class Robot: public SampleRobot
{
	Preferences * prefs;
	double pref1, pref2, pref3;
	PowerDistributionPanel* m_pdp;

public:
	Robot():
		prefs(Preferences::GetInstance()),
		pref1(0),
		pref2(0),
		pref3(0)
	{
		printf("\n\n\nIn Robot Constructor\n\n\n");
		ID
		// must wait some ?amount? of time for the Preferences GetInstance to run in the background
		// otherwise the Gets won't see the previously saved preferences and the default values are used.
		Wait(2.0); 	// documents forgot to tell me about this gotcha
	}

	~Robot()
	{
		printf("\n\n\nIn Robot Destructor\n\n\n");
		Wait(2.0);
	}

	void RobotInit()
	{
		printf("\n\n\nIn RobotInit\n\n\n");

		std::string message;

		// preferences may be viewed and changed programmatically and
		// manually on the SmartDashBoard / View / Add / Robot Preferences

		bool Done = false;
		SmartDashboard::PutBoolean("Check If You Are Done", Done);

		while ( ! SmartDashboard::GetBoolean("Check If You Are Done", false) && ! IsEnabled() )
		{
			pref1 = prefs->GetDouble("MyPref1", 1.0);
			pref2 = prefs->GetDouble("MyPref2", 1.0);
			pref3 = prefs->GetDouble("MyPref3");

			printf("Preferences RobotInit  %f  %f  %f\n\n", pref1, pref2, pref3);

			message = "";
			if ( pref1 < 0 || pref1 > 4 )	message += "MyPref1 not between 0 and 4; ";
			if ( pref2 != 0 && pref2 != 1 )	message += "MyPref2 not = 0 or 1; ";
			if ( pref3 > 100. )				message += "MyPref3 not < 100.; ";
			if ( message == "" )			message = "All Preferences Valid - Check Done To Continue";
			SmartDashboard::PutString("Preference Message ", message);

			Wait(1.);
		}
	}

	void Autonomous()
	{
		printf("\n\n\nIn Autonomous\n\n\n");

		pref1 = prefs->GetDouble("MyPref1", 1.0);
		pref2 = prefs->GetDouble("MyPref2", 2.0);
		pref3 = prefs->GetDouble("MyPref3", 3.0);

		printf("Preferences Autonomous  %f   %f   %f\n\n", pref1, pref2, pref3);
	}

	void Disabled()
	{
		printf("\n\n\nIn Disabled\n\n\n");
	}

	void OperatorControl()
	{
		printf("\n\n\nIn OperatorControl\n\n\n");
	m_pdp->ClearStickyFaults();
		printf("m_pdp Sticky faults cleared\n");
		//fflush(NULL);
		temp = m_pdp->GetTemperature();
		printf("%f %f %f %f %f\n",
			m_pdp->GetTemperature(), m_pdp->GetTotalCurrent(), m_pdp->GetTotalEnergy(), m_pdp->GetTotalPower(), m_pdp->GetVoltage());
		while (IsOperatorControl() && IsEnabled())
		{
			printf("Preferences OperatorControl  %f   %f   %f\n\n", pref1, pref2, pref3);
		printf("Running OperatorControl\n");
			//fflush(NULL);
			printf("x y z Gs %f %f %f\n", PullingGs->GetX(), PullingGs->GetY(), PullingGs->GetZ() );
			//fflush(NULL);
			printf("%f %f %f %f %f\n",
				m_pdp->GetCurrent(15), m_pdp->GetTotalCurrent(), m_pdp->GetTotalEnergy(), m_pdp->GetTotalPower(), m_pdp->GetVoltage());
			//fflush(NULL);
			// Get the current going through channel 7, in Amperes.
			// The PDP returns the current in increments of 0.125A.
			// At low currents the current readings tend to be less accurate.
//			SmartDashboard::PutNumber("Current Channel 7", m_pdp->GetCurrent(7));
			// Get the voltage going into the PDP, in Volts.
			// The PDP returns the voltage in increments of 0.05 Volts.
//			SmartDashboard::PutNumber("Voltage", m_pdp->GetVoltage());
			// Retrieves the temperature of the PDP, in degrees Celsius.
//			SmartDashboard::PutNumber("Temperature", m_pdp->GetTemperature());

			if(Xbox->GetRawButton(1) == true)
				mMotor->Set(0.5);
			else if(mSwitch->Get() == true)
				mMotor->Set(1.0);
			else if(GetUserButton() == true)
				mMotor->Set(-0.5);
			else
				mMotor->Disable();
			// Get the current going through channel 7, in Amperes.
			// The PDP returns the current in increments of 0.125A.
			// At low currents the current readings tend to be less accurate. 
			SmartDashboard::PutNumber("Current Channel 7", m_pdp->GetCurrent(7));
			// Get the voltage going into the PDP, in Volts.
			// The PDP returns the voltage in increments of 0.05 Volts.
			SmartDashboard::PutNumber("Voltage", m_pdp->GetVoltage());
			// Retrieves the temperature of the PDP, in degrees Celsius.
			SmartDashboard::PutNumber("Temperature", m_pdp->GetTemperature());
			Wait(kUpdatePeriod);
			Wait(1.);
		}
	}

};

START_ROBOT_CLASS(Robot);
