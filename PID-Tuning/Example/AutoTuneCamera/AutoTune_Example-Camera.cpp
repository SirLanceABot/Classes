// Automatically Tune PID controller with relay method

#include "WPILib.h"
#include "AutoTune_Example.h"
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
double controllerMin = -400.L, controllerMax = 400.L;

// sign correlation of control signal and process variable
int Direction=DIRECT; // DIRECT is + controller is + process variable; REVERSE is the other way + yields -

// controller parameters used in case tuning isn't first
double kp=0.L, ki=0.L, kd=0.L;
//Control with Kp= 8.48827 Ki=5.41516 Kd=3.32634 Ku=14.1471 Pu=3.135
//Control with Kp= 23.506 Ki=28.8063 Kd=4.79522 Ku=39.1766 Pu=1.632

// specify percentage of process range to use to define the proportional band within which to control with the PID
// band is supposed to be set point + and - percentage of the process variable range
// if band too small, system oscillates; if band too big, might have the integration windup to deal with
//double ProportioningBand = 20.;  // percentage of process range to use as one side of the proportioning

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

//                              M A I N

int TuneMain(CANTalon * mMotorA, CANTalon * mMotorB)
{
std::cout << __FILE__ << " " <<__DATE__ << " " << __TIME__ << " " << __PRETTY_FUNCTION__ << " line:" << __LINE__ << std::endl;

std::shared_ptr<NetworkTable> roboRealm;
roboRealm = NetworkTable::GetTable("RoboRealm");

timer = new Timer();

DriverStation& mDS = DriverStation::GetInstance();

// globals to initialize since they are set during run
tuning = true; //false just runs the PID controller - no tuning first;

// timer for entire run
timer->Start();

/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//                  P R O C E S S   V A R I A B L E S

setpoint = 0.0f;  // center of range - no power, no turning
input = setpoint;  // initialize because the tuner has a logic flaw; it starts using this input value before there has been any response from the system to the output
output = 0.0f;  // corresponding power control signal for steady state at setpoint; center of range to start tuning

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

Wait(.1f); // wait for console output to catch up

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

Wait(1.); // wait for system to quiet down from calibration run

mMotorA->SetEncPosition(0); // reset A & B encoders
mMotorB->SetEncPosition(0);

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

	double imageWidth = roboRealm->GetNumber("IMAGE_WIDTH", 320); // fixme: hard-wired 320/2 throughout this program
	double xPosition;

	while (roboRealm->GetNumber("BLOB_COUNT", 0.L) < .5L) { Wait(.03); }; // wait for a blob to reappear

	xPosition = roboRealm->GetNumber("COG_X", 160.L); // get the center of gravity of the blob

	input = imageWidth/2.L - xPosition; // positive deviation means object too far left.  positive power makes deviation more positive-direct control

	if(!tuning && abs(input) <= 2)
		{
		std::cout << "   O N   T A R G E T !";
		// stop driving
		mMotorA->SetControlMode(mMotorA->ControlMode::kPercentVbus);
		mMotorA->Set(0.f);
		mMotorB->SetControlMode(mMotorB->ControlMode::kPercentVbus);
		mMotorB->Set(0.f);
		myPID.SetMode(MANUAL);
		break;
		}

/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
	\/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

	if(tuning) // running the tuning in the loop
	{
		int val = (aTune.Runtime()); // 0 took a time step; 1 done tuning; 2 called faster than sample rate
		if (val == 1) tuning = false;  // tuning just completed
		else if (val == 2) {std::cout << std::endl; continue;} // skipping this step
		
		if(!tuning) // we had been tuning but check to see if the last tune call completed the tuning
			{
			// we're done tuning, set the tuning parameters in the controller
				kp = aTune.GetKp();
				ki = aTune.GetKi();
				kd = aTune.GetKd();
				std::cout << std::endl << "Control with Kp= " << kp << " Ki=" << ki << " Kd=" << kd
					 << " Ku=" << aTune.GetKu() << " Pu=" << aTune.GetPu()<< std::endl << std::endl << "                    ";
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
//		// check for in or out of the proportioning band
//		// this implementation doesn't have the exact process variable range so fake band
//		// by assuming the set point is in the center of the range and not zero and the range is positive
//		if (input < (1. - 2.*ProportioningBand/100.) * setpoint) {
//			myPID.SetMode(MANUAL);
//			output = controllerMax;  // way too slow so full on
//		}
//		else if (input > (1. + 2.*ProportioningBand/100.) * setpoint) {
//			myPID.SetMode(MANUAL);
//			output = controllerMin;  // way too fast so full off
//		}
//		else{
			myPID.SetMode(AUTOMATIC);
			if (!myPID.Compute()){continue;} // get the new control signal from the controller if it's available
			// if the negation of the return value is true, not enough time has elapsed or it's in manual which it can't be since it's in band
//	}
	}

/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
	\/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//    S E N D   T H E   N E W   C O N T R O L   S I G N A L   T O   T H E   P R O C E S S

//   Tank turn so control left and right treads in opposite directions

	mMotorA->Set(output);
	mMotorB->Set(-output);

	// clear I accumulation from previous setpoint
	if (output != output_previous)
		{
		mMotorA->ClearIaccum();
		mMotorB->ClearIaccum();
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
	else              {                std::cout << " slow loop by:" << waitTime << std::endl;}
	
	if(millis() > 100000)
		break;
}
/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//                     P R O C E S S   L O O P   E N D


std::cout << std::endl;
mMotorA->Set(0.f); // stop driving
mMotorB->Set(0.f);

mMotorA->Reset();
mMotorB->Reset();
Wait(.5L);

mMotorA->Enable();
mMotorB->Enable();
Wait(.5L);

mMotorA->ClearIaccum();
mMotorB->ClearIaccum();

return 0;
}
/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//                            M A I N   E N D

