// Aim robot at target using camera deviation input to PID controller to set tank turn track speed

#include "CameraTargetingPID.h"
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

// controller parameters
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
/*          declarations         */

// process dependent values
double sampleTime = .06L; // seconds

/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//                              M A I N

int TargetMain(CANTalon * mFrontLeftMotor, CANTalon * mFrontRightMotor, CANTalon * mRearLeftMotor, CANTalon * mRearRightMotor)
{
std::cout << __FILE__ << " " <<__DATE__ << " " << __TIME__ << " " << __PRETTY_FUNCTION__ << " line:" << __LINE__ << std::endl;

std::shared_ptr<NetworkTable> roboRealm;
roboRealm = NetworkTable::GetTable("RoboRealm");

timer = new Timer();

DriverStation& mDS = DriverStation::GetInstance();

std::shared_ptr<DigitalOutput> mLight;
mLight = std::make_shared<DigitalOutput>(DIO_PORT::LIGHT_RING);
mLight->Set(true);

//Wait(600.); // testing camera view with the lights on
//return 0;

// timer for entire run
timer->Start();

/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//                  P R O C E S S   V A R I A B L E S

setpoint = 0.0f;  // center of range - no power, no turning

/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//   S E T U P   S T R I P C H A R T   T O   D I S P L A Y   A C T I O N S

StripChart myChart(	setpoint,		// center of left process variable graph
					controllerMin,	// minimum value of right controller graph
					controllerMax);	// maximum value of right controller graph

/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//             S E T U P   T H E   C O N T R O L L E R

kp = TALON_DRIVE::CameraKp;
ki = TALON_DRIVE::CameraKi;
kd = TALON_DRIVE::CameraKd;

PID myPID(&input, &output, &setpoint, &loopStartTime, kp, ki, kd, Direction);
myPID.SetSampleTime((int)(1000.L*sampleTime + .5L)); // input sampling time; controller checks to make sure it wasn't called faster than input changes
myPID.SetOutputLimits(controllerMin, controllerMax);
myPID.SetControllerDirection(Direction);

/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//                    S E T U P   T H E   L O O P

 mFrontLeftMotor->SetEncPosition(0); // reset A & B encoders
 mFrontRightMotor->SetEncPosition(0);

timer->Reset(); // reset the loop timer to 0

/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//                       P R O C E S S   L O O P

myPID.SetMode(AUTOMATIC);

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

	if(abs(input) <= 2) // pick so robot will coast to the center - no more, no less
		{
		// stop driving
		std::cout << "   O N   T A R G E T !";
		myPID.SetMode(MANUAL);
		break;
		}

/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
	\/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

	if (!myPID.Compute()){continue;} // get the new control signal from the controller if it's available
	// if the negation of the return value is true, not enough time has elapsed or it's in manual which it can't be since it's in band

/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
	\/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//    S E N D   T H E   N E W   C O N T R O L   S I G N A L   T O   T H E   P R O C E S S

//   Tank turn so control left and right treads in opposite directions

	 mFrontLeftMotor->Set(output);
	 mFrontRightMotor->Set(-output);

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
	else              {std::cout << " slow loop by:" << waitTime << std::endl;}
	
	if(millis() > 100000)
		break;
}
/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//                     P R O C E S S   L O O P   E N D

// stop driving, leave PID mode, set all back to %VBus for next by ArcadeDrive, etc, in WPILib driving
 mFrontLeftMotor->SetControlMode( mFrontLeftMotor->ControlMode::kPercentVbus);
 mFrontLeftMotor->Set(0.f);
 mFrontRightMotor->SetControlMode( mFrontRightMotor->ControlMode::kPercentVbus);
 mFrontRightMotor->Set(0.f);
 mRearLeftMotor->SetControlMode( mRearLeftMotor->ControlMode::kPercentVbus);
 mRearLeftMotor->Set(0.f);
 mRearRightMotor->SetControlMode( mRearRightMotor->ControlMode::kPercentVbus);
 mRearRightMotor->Set(0.f);
 mFrontLeftMotor->ClearIaccum(); // clean up anything left over from any previous Talon PID controller
 mFrontRightMotor->ClearIaccum();

 Wait(.2);
 
// if going back to driving with WPILib, say ArcadeDrive then do NOT use the SetInverted(true) because ArcadeDrive already inverts the right motor.
// if doing your own drive commands - Set(...) then the SetInverted is probably needed.  It was false during PID control because PID control has its
// own inversion and another SetInverted(true) negates that in bad ways.  Done with the targeting PID control so consider need for SetInverted(true).

//mFrontRightMotor->SetInverted(true);  // invert power so + power goes forward for both sides; not used for PID control of mirrored motors; only for %VBus
//mRearRightMotor->SetInverted(true);  // invert power so + power goes forward for both sides; not used for PID control of mirrored motors; only for %VBus

// mFrontLeftMotor->Reset();
// mFrontRightMotor->Reset();
//Wait(.5L);
//
// mFrontLeftMotor->Enable();
// mFrontRightMotor->Enable();
//Wait(.5L);
//
// mFrontLeftMotor->ClearIaccum();
// mFrontRightMotor->ClearIaccum();

mLight->Set(false);

std::cout << "\n\nExit TargetMain\n\n" << std::endl;

return 0;
}
/*  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\  /\
   //\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\
   \\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//\\//
    \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/  \/ */

//                            M A I N   E N D

