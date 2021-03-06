// Automatically Tune PID controller with relay method

#include "WPILib.h"
#include "Constants.h"
#include "AutoTune-tank-turn.h"
#include "StripChart.h"
#include "stats_running.h" // statistical package
#include <math.h>
#include <sstream>

/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//             M I L L I -- S E C O N D   T I M E R

Timer *timer;

unsigned long millis(void); // prototype for get system time

unsigned long millis(void)  // get current time in milliseconds
{
	return (unsigned long) (timer->Get() * 1000.L + .5f);
}

/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//            C O N T R O L L E R    D E C L A R A T I O N S

#include "PID-ARDUINO.h"

double setpoint;  //  process value setpoint - "input" (motor speed from encoder) controlled to "setpoint" (desired value of speed) by changing "output" (motor power)
double output;    //  control value

// control (output) signal limits
double controllerMin = -1.L, controllerMax = 1.L; // range of the Talon %VBus control mode

// sign correlation of control signal and process variable
int Direction=DIRECT; // DIRECT is + controller is + process variable; REVERSE is the other way + yields -

// controller parameters used in case tuning isn't first
double kp=0.L, ki=0.L, kd=0.L;

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
double sampleTime = .011L; // seconds
double aTuneStartValue;    // initial control signal (controller output) used for tuning; the output at the first time step is what is actually used to initialize the tuner
double aTuneStep=.25L;     // + and - for max and min from the initial control signal for relay value
double aTuneNoise=0.L;     // + or - noise on set point process variable to ignore in same units as the PV - deadband on trigger
int aTuneLookBack=3;       // number of samples - don't exceed the period of the process with aTuneLookBack*sampleTime

/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//                              M A I N

int TuneMain(CANTalon * mMotorA, CANTalon * mMotorB, CANTalon * mMotorC, CANTalon * mMotorD, bool TankTurn) // true for B side tank turn opposed to A side or false for B side to drive straight with A side
{
std::cout << __FILE__ << " " <<__DATE__ << " " << __TIME__ << " " << __PRETTY_FUNCTION__ << " line:" << __LINE__ << std::endl;

timer = new Timer();

DriverStation& mDS = DriverStation::GetInstance();

RunningStats ADriveEncoder, BDriveEncoder;

double ADriveEncoderMean, BDriveEncoderMean, A_B_Ratio;

// globals to initialize since they are set during run
tuning = true; //false just runs the PID controller - no tuning first;

// timer for entire run
timer->Start();

/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//             S T R A I G H T -- L I N E   C A L I B R A T I O N


// determine average speeds from encoders to get A/B calibration (determine power to match speeds)
// tune A and have B follow with estimated power for the tank turn

// start both drives at the same power
// best first guess is same power for same speed on both

mMotorA->SetEncPosition(0); // reset A & B drive motor encoders
mMotorB->SetEncPosition(0);

Wait(.02L);

double testPower = .5L;  // turn on motors
mMotorA->Set(testPower);
mMotorB->Set(testPower);

std::ostringstream buffer;
buffer << "Motor A is CAN Talon:";
mMotorA->GetDescription(buffer);
buffer << "\nMotor B is CAN Talon:";
mMotorB->GetDescription(buffer);
std::cout << "\n\nTuning Motor A; Motor B follows with reverse power to perform the tank turn\n" << buffer.str() << "\n\n";

std::cout << "1.2 second Straight-line Calibration run;  A Power output:" << testPower << "    B Power output:" << testPower << "\n";

printf("millisecs                 Lspeed    Lmean    Lstddev    Lmin     Lmax                          Rspeed    Rmean    Rstddev    Rmin    Rmax     A/B\n");

timer->Reset();  // reset straight run calibration timer to 0

// get the encoders' speeds and print running statistics of response to both from the same motor power input
for (int count = 1; count <= 20; count++)
{
	double ASpeed, BSpeed;

	ASpeed  = mMotorA->GetSpeed();  // get the new signal from the A encoder
	BSpeed = mMotorB->GetSpeed();  // get the new signal from the B encoder

	if(count < 10) // skip beginning transient; may or may not be typical ratio of the 2 motor speeds
	{
		printf("%lu ; A speed encoder: %f ; B speed encoder: %f\n",
		millis(),
		ASpeed,
		BSpeed);
	}
	else
	{ // Guess that ratio is steady state now so collect data
	ADriveEncoder.Push(ASpeed);
	BDriveEncoder.Push(BSpeed);
	
	ADriveEncoderMean = fabs(ADriveEncoder.Mean()); // abs accommodates backwards motors or encoders
	BDriveEncoderMean = fabs(BDriveEncoder.Mean());
	
	printf("%lu ; A speed encoder: %f %f %f %f %f ; B speed encoder: %f %f %f %f %f\n",
		millis(),
		ASpeed , ADriveEncoderMean , ADriveEncoder.StandardDeviation() , ADriveEncoder.Minimum() , ADriveEncoder.Maximum(),
		BSpeed , BDriveEncoderMean , BDriveEncoder.StandardDeviation() , BDriveEncoder.Minimum() , BDriveEncoder.Maximum());
	printf("v, v, a, t A: %f, %f, %f,%f;  B: %f, %f, %f,%f;  C: %f, %f, %f,%f;  D: %f, %f, %f,%f\n",
		 mMotorA->GetBusVoltage(), mMotorA->GetOutputVoltage()	, mMotorA->GetOutputCurrent(), mMotorA->Get(),
		 mMotorB->GetBusVoltage(), mMotorB->GetOutputVoltage()	, mMotorB->GetOutputCurrent(), mMotorB->Get(),
		 mMotorC->GetBusVoltage(), mMotorC->GetOutputVoltage()	, mMotorC->GetOutputCurrent(), mMotorC->Get(),
		 mMotorD->GetBusVoltage(), mMotorD->GetOutputVoltage()	, mMotorD->GetOutputCurrent(), mMotorD->Get());
	}

	Wait(0.1L);
}

mMotorA->Set(0.f); // stop motors - done driving straight for the calibration run
mMotorB->Set(0.f);

ADriveEncoder.Print(stdout, "A Drive Encoder"); // print results of calibration run
BDriveEncoder.Print(stdout, "B Drive Encoder");
std::cout << "\n";

if (ADriveEncoderMean == 0.)
{ // tuning A side so need the encoder working on that side
	std::cout << "\n\nA drive train encoder value is 0; terminating tuning\n\n";
	return 0;
}

if (BDriveEncoderMean == 0.) BDriveEncoderMean = ADriveEncoderMean; // if B side encoder is dead we can keep tuning the A side as long as B is similar to A
A_B_Ratio = ADriveEncoderMean/BDriveEncoderMean;

if (TankTurn) A_B_Ratio = -A_B_Ratio; // negative sign to indicate turn turn or + sign is straight driving for the B side motor

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
// boost A if it's running slower than B; assumes we'll tune both motors individually and we want to try to match the tuning speeds
if(fabs(A_B_Ratio) < 1.L) aTuneStep = aTuneStep/fabs(A_B_Ratio); // abs to wipe the tank/straight +/- sign
aTune.SetOutputStep(aTuneStep);
aTune.SetSampleTime((int)(1000.L*sampleTime + .5L));  // round to a millisecond after float math so arithmetic comparisons succeed
aTune.SetLookbackTime((int)(aTuneLookBack*1000.L*sampleTime + .5L)); // sample look back in milliseconds - don't exceed the period of the process
std::cout << "A Power output at setpoint: " << output << "    B Power output at setpoint: " << output*A_B_Ratio << "\n";
std::cout << "A rate mean: " << ADriveEncoderMean << "   B rate mean: " << BDriveEncoderMean << "\n";

/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//   S E T U P   S T R I P C H A R T   T O   D I S P L A Y   A C T I O N S

StripChart myChart(	setpoint,		// center of left process variable graph
					controllerMin,	// minimum value of right controller graph
					controllerMax);	// maximum value of right controller graph

Wait(.5f); // wait for console output to catch up and motors to stop

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

	input = mMotorA->GetSpeed();  // get the new signal from the A encoder

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
			for (int doTwice = 1; doTwice <=2; doTwice++)
			{
				if(doTwice == 1) aTune.SetControlType(PID_CONTROL); // just to see the PID values, but ...
				if(doTwice == 2) aTune.SetControlType(PI_CONTROL); // 2nd time through no D because secondary controllers in a cascade don't usually have a D

			// we're done tuning, set the tuning parameters in the controller
				kp = aTune.GetKp();
				ki = aTune.GetKi();
				kd = aTune.GetKd();
				std::cout << "\n\nControl with Kp= " << kp << " Ki=" << ki << " Kd=" << kd
					 << " Ku=" << aTune.GetKu() << " Pu=" << aTune.GetPu() << "\n";
				myPID.SetTunings(kp,ki,kd); // use new parameters

//////////////////////////////////////////////////////////////////////////////////
//            START Conversion to CAN Talon units
//
				kp = kp * TALON_DRIVE::KuUnits;
				ki = ki * TALON_DRIVE::KuUnits/TALON_DRIVE::PuUnits;
				kd = kd * TALON_DRIVE::KuUnits*TALON_DRIVE::PuUnits;
				std::cout << "\nCAN Talon Control with Kp= " << kp << " Ki=" << ki << " Kd=" << kd << "\n";
//
//            END Conversion to CAN Talon units
/////////////////////////////////////////////////////////////////////////////////

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

	// use the new control signal for A; B is estimated power (control signal) difference needed for same speed from above calibration run
	//  In Speed mode, output value is in position change / 10ms.
	mMotorA->Set(output);
	mMotorB->Set(output/A_B_Ratio);

	// chart the action
	std::cout << "\n";
	myChart.PrintStripChart(loopStartTime,	// time in milliseconds
							input,			// process variable - input to controller
							output);		// output from controller
	
	// pace the loop time - aim for fixed sampling rate
	waitTime = sampleTime - (double)(millis() - loopStartTime)/1000. + .00005; // add a little for tiny timing errors
	if (waitTime > 0) {Wait(waitTime); /*std::cout << " fast loop by:" << waitTime;*/}
	else              { std::cout << " slow loop by:" << waitTime << "\n";}
	
	if(millis() > 3500)
		break;
}
/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//                     P R O C E S S   L O O P   E N D

std::cout << std::endl;
mMotorA->Set(0.); // stop driving
mMotorB->Set(0.);

return 0;
}
/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//                            M A I N   E N D

