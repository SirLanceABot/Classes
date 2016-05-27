// Automatically Tune Camera Targeting PID controller with relay method

#include "AutoTuneCamera.h"
#include "WPILib.h"
#include "constants.h"
#include "StripChart.h"
#include <math.h>

/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//             M I L L I -- S E C O N D   T I M E R

Timer *timer;

unsigned long millis(void); // prototype for get system time

unsigned long millis(void)  // get current time in milliseconds
{
	return (unsigned long) (timer->Get() * 1000.L + .5L);
}

/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//            C O N T R O L L E R    D E C L A R A T I O N S

#include "PID-ARDUINO.h"

double setpoint;  //  process value setpoint - "input" (motor speed from encoder) controlled to "setpoint" (desired value of speed) by changing "output" (motor power)
double output, output_previous = -DBL_MAX;    //  control value

// control (output) signal limits
double controllerMin = TALON_DRIVE::CONTROLLER_MIN, controllerMax = TALON_DRIVE::CONTROLLER_MAX;

// sign correlation of control signal and process variable
int Direction=DIRECT; // DIRECT is + controller is + process variable; REVERSE is the other way + yields -

// controller parameters used in case tuning isn't first
double kp=0.L, ki=0.L, kd=0.L;

/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//                P R O C E S S   V A R I A B L E S

unsigned long loopStartTime;  // system time in milliseconds at start of a step for tuning or controlling

double input;                 //  process value from sensor (encoder)

/* *********************************** */
/* *********************************** */
/*         tuning declarations         */
#include "PID_AutoTune-ARDUINO.h"

int tuning; //false just runs the PID controller - no tuning first;

// process dependent values for tuning - don't destroy the process with excess max/min
double sampleTime = .06L; // seconds
double aTuneStartValue;    // initial control signal (controller output) used for tuning; the output at the first time step is what is actually used to initialize the tuner
double aTuneStep = 100.L;     // + and - for max and min from the initial control signal for relay value
double aTuneNoise = 0.L;     // + or - noise on set point process variable to ignore in same units as the PV - deadband on trigger
int aTuneLookBack = 6;       // number of samples - don't exceed the period of the process with aTuneLookBack*sampleTime

/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//                        T U N E   M A I N

int TuneMain(CANTalon * mFrontLeftMotor, CANTalon * mFrontRightMotor, CANTalon * mRearLeftMotor, CANTalon * mRearRightMotor)
{
std::cout << __FILE__ << " " <<__DATE__ << " " << __TIME__ << " " << __PRETTY_FUNCTION__ << " line:" << __LINE__ << std::endl;

std::cout << "\n\nSetting PID and follower modes\n\n";

std::shared_ptr<DigitalOutput> mLight;
mLight = std::make_shared<DigitalOutput>(DIO_PORT::LIGHT_RING);
mLight->Set(true);

//Wait(600.); // testing camera view with the lights on
//return 0;

// Setup the tread drive motors' internal (Talon SRX) PID controller to be the secondary controller in a cascade of controllers
mFrontLeftMotor->ClearIaccum(); // clean up anything left over from any previous Talon PID controller
mFrontRightMotor->ClearIaccum();

// PID Tuning parameters go here
mFrontLeftMotor->SetPID(TALON_DRIVE::TankTurnKp, TALON_DRIVE::TankTurnKi, TALON_DRIVE::TankTurnKd);
mFrontRightMotor->SetPID(TALON_DRIVE::TankTurnKp, TALON_DRIVE::TankTurnKi, TALON_DRIVE::TankTurnKd);

mFrontLeftMotor->SetControlMode(mFrontLeftMotor->ControlMode::kSpeed);
mFrontLeftMotor->Set(0.);

mFrontRightMotor->SetControlMode(mFrontRightMotor->ControlMode::kSpeed);
mFrontRightMotor->Set(0.);
Wait(.2);

mRearLeftMotor->SetControlMode(CANSpeedController::kFollower);
mRearLeftMotor->Set(CAN_PORT::FRONT_LEFT);

mRearRightMotor->SetControlMode(CANSpeedController::kFollower);
mRearRightMotor->Set(CAN_PORT::FRONT_RIGHT);
Wait(.2);

// right motor is mirror of left so invert its actions since it runs backwards of the left motor
mFrontRightMotor->SetInverted(true);  // invert power so + power goes forward for both sides; not used for PID control of mirrored motors; only for %VBus
mFrontRightMotor->SetSensorDirection(true); // invert encoder to match left; true reverses GetPosition & GetSpeed but not GetEncPosition nor GetEncVel
mFrontRightMotor->SetClosedLoopOutputDirection(true); // reverses the power to the motor in PID controller mode - SetInverted doesn't work for PID
Wait(.2);

// setup the roboRealm linkage
std::shared_ptr<NetworkTable> roboRealm;
roboRealm = NetworkTable::GetTable("RoboRealm");

// DriverStation linkage for IsEnabled()
DriverStation& mDS = DriverStation::GetInstance();

// globals to initialize since they are set during run
tuning = true; //false just runs the PID controller - no tuning first;

// timer for entire run
timer = new Timer();
timer->Start();

/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//                  P R O C E S S   V A R I A B L E S

setpoint = 0.L;  // center of range - no power, no turning
input = setpoint;  // initialize because the tuner has a logic flaw; it starts using this input value before there has been any response from the system to the output
output = 0.L;  // corresponding power control signal for steady state at setpoint; center of range to start tuning

/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//                   S E T U P   T H E   T U N E R

PID_ATune aTune(&input, &output, &setpoint, &loopStartTime, Direction);
aTune.SetControlType(PID_CONTROL);
aTune.SetNoiseBand(aTuneNoise);
aTune.SetOutputStep(aTuneStep);
aTune.SetSampleTime((int)(1000.L*sampleTime + .5L));  // round to a millisecond after float math so arithmetic comparisons succeed
aTune.SetLookbackTime((int)(aTuneLookBack*1000.L*sampleTime + .5L)); // sample look back in milliseconds - don't exceed the period of the process

/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//   S E T U P   S T R I P C H A R T   T O   D I S P L A Y   A C T I O N S

StripChart myChart(	setpoint,		// center of left process variable graph
					controllerMin,	// minimum value of right controller graph
					controllerMax);	// maximum value of right controller graph

Wait(.1f); // maybe RoboRealm and other stuff needs to settle

// Setup the controller to use immediately after tuning determines new parameters //////
PID myPID(&input, &output, &setpoint, &loopStartTime, kp, ki, kd, Direction);
myPID.SetSampleTime((int)(1000.L*sampleTime + .5L)); // input sampling time; controller checks to make sure it wasn't called faster than input changes
myPID.SetOutputLimits(controllerMin, controllerMax);
myPID.SetControllerDirection(Direction);

/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//                    S E T U P   T H E   L O O P

std::cout << "Starting Tuning Loop" << std::endl;

Wait(1.);

mFrontLeftMotor->SetEncPosition(0); // reset A & B encoders
mFrontRightMotor->SetEncPosition(0);

timer->Reset(); // reset the loop timer to 0

/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//                       P R O C E S S   L O O P

while (mDS.IsEnabled())
{
	double waitTime;
	
	loopStartTime = millis();  // time we get the input and take a step for tuning or controlling

/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
	\/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

  //    G E T   T H E   C U R R E N T   P R O C E S S   V A R I A B L E

	double imageWidth = roboRealm->GetNumber("IMAGE_WIDTH", 320); // fixme: hard-wired 320/2 = 160 throughout this program
	double xPosition;

	while (roboRealm->GetNumber("BLOB_COUNT", 0.L) < .5L) { Wait(.01); }; // wait for a blob to reappear

	xPosition = roboRealm->GetNumber("COG_X", 160.L); // get the center of gravity of the blob

	input = imageWidth/2.L - xPosition; // positive deviation means object too far left.  positive power makes deviation more positive-direct control

	if(!tuning && abs(input) <= 2) // pick so robot will coast to the center - no more, no less
		{
		std::cout << "   O N   T A R G E T !";
		myPID.SetMode(MANUAL);
		break;
		}

/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
	\/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

	if(tuning) // running the tuning in the loop
	{
		switch (aTune.Runtime()) // 0 took a time step; 1 done tuning; 2 called faster than sample rate; 3 intermediate parameters available
		{
		case 1:  // tuning just completed; mark that event, print the peak, and move on
			tuning = false;
		case 3:  // time step okay, print the peak, and process time step
			printf("  peaks at %.3f-%.3f, Ku=%.5f, Pu=%.5f", aTune.GetPeak_1(), aTune.GetPeak_2(), aTune.GetKu(), aTune.GetPu());
		case 0:  // time step okay, process time step
			break;
		case 2:  // too fast, skipping this step
			continue;
		default:
			printf("\n\nUnknown return from Runtime()\n\n");
		}
		
		if(!tuning) // we had been tuning but check to see if the last tune call completed the tuning
			{
			// we're done tuning, set the tuning parameters in the controller
				kp = aTune.GetKp();
				ki = aTune.GetKi();
				kd = aTune.GetKd();
				std::cout << "\n\nControl with Kp= " << kp << " Ki=" << ki << " Kd=" << kd
					 << " Ku=" << aTune.GetKu() << " Pu=" << aTune.GetPu() << "\n";
				myPID.SetTunings(kp,ki,kd); // use new parameters

// "setpoint" still good since it was set above for tuning
// "output" was restored by the autotuner completing successfully and is good for the setpoint
// since we started autotuning at steady state
// motor speed will be set but PID controller not invoked on this loop iteration.
// That's OK since we are returning to the pre-autotuning state.
			}
	}
	else // now running the PID normally in the loop
	{ 
		myPID.SetMode(AUTOMATIC);
		if (!myPID.Compute()){continue;} // get the new control signal from the controller if it's available
		// if the negation of the return value is true, not enough time has elapsed or it's in manual which it can't be
	}

/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
	\/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//    S E N D   T H E   N E W   C O N T R O L   S I G N A L   T O   T H E   P R O C E S S

//   Tank turn so control left and right treads in opposite directions

	mFrontLeftMotor->Set(output);
	mFrontRightMotor->Set(-output);

	Wait(.01);
	printf(" Set +/- %f; Get %f %f %f %f ", output, mFrontLeftMotor->Get(), mFrontRightMotor->Get(), mRearLeftMotor->Get(), mRearRightMotor->Get());
	
	// clear I accumulation from previous setpoint
	if (output != output_previous)
		{
		mFrontLeftMotor->ClearIaccum();
		mFrontRightMotor->ClearIaccum();
		output_previous = output;
		}

	/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
	   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
	   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
		\/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

	// chart the action
	std::cout << "\n";
	myChart.PrintStripChart(loopStartTime,	// time in milliseconds
							input,			// process variable - input to controller
							output);		// output from controller
	
	// pace the loop time - aim for fixed sampling rate
	waitTime = sampleTime - (double)(millis() - loopStartTime)/1000. + .00005; // add a little for tiny timing errors
	if (waitTime > 0) {Wait(waitTime); /*std::cout << " fast loop by:" << waitTime;*/}
	else              {std::cout << " slow loop by:" << waitTime << "\n";}
	
	if(millis() > 100000)
		break;
}
/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//                     P R O C E S S   L O O P   E N D E D

// stop driving immediately; put all motors back to %VBus mode and make speed 0

std::cout << "\n\nSetting %VBus modes\n\n";

mFrontLeftMotor->SetControlMode(mFrontLeftMotor->ControlMode::kPercentVbus);
mFrontLeftMotor->Set(0.f);
mFrontRightMotor->SetControlMode(mFrontRightMotor->ControlMode::kPercentVbus);
mFrontRightMotor->Set(0.f);
mRearLeftMotor->SetControlMode(mRearLeftMotor->ControlMode::kPercentVbus);
mRearLeftMotor->Set(0.f);
mRearRightMotor->SetControlMode(mRearRightMotor->ControlMode::kPercentVbus);
mRearRightMotor->Set(0.f);

// if going back to driving with WPILib, say ArcadeDrive then do NOT use the SetInverted(true) because ArcadeDrive already inverts the right motor.
// if doing your own drive commands - Set(...) then the SetInverted is probably needed.  It was false during PID control because PID control has its
// own inversion and another SetInverted(true) negates that in bad ways.  Done with the targeting PID control so consider need for SetInverted(true).

//mFrontRightMotor->SetInverted(true);  // invert power so + power goes forward for both sides; not used for PID control of mirrored motors; only for %VBus
//mRearRightMotor->SetInverted(true);  // invert power so + power goes forward for both sides; not used for PID control of mirrored motors; only for %VBus
		
mFrontLeftMotor->ClearIaccum(); // clean up for next PID usage
mFrontRightMotor->ClearIaccum();

Wait(.1);

mLight->Set(false);

std::cout << "\n\nExit TuneMain\n\n" << std::endl;

return 0;
}
/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//                     T U N E   M A I N   E N D E D

