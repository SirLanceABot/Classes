#ifndef CONSTANTS_H_
#define CONSTANTS_H_

// todo: Select one chassis

//#define TABLE_TOP_PG45775126000_26_9KET
#define ROBOT_CIMS

#if defined (TABLE_TOP_PG45775126000_26_9KET)
	#if defined (ROBOT_CIMS)
		#error "More than 1 chassis defined"
	#else
		#warning "using TABLE_TOP_PG45775126000_26_9KET"
	#endif
#elif defined (ROBOT_CIMS)
		#warning "using ROBOT_CIMS"
#else
	#error "No chassis defined"
#endif

namespace TALON_DRIVE
{
#ifdef TABLE_TOP_PG45775126000_26_9KET
// 7 encoder pulses/motor rev x 26.9:1 motor revs/gear box output shaft rev = 188; times 4x quadrature encoder = 753.2 edges/motor rev/output shaft rev
// max GetSpeed about 260
const uint16_t ENCODER_CODES_PER_REV = 188;
const double CONTROLLER_MIN = -260.L;
const double CONTROLLER_MAX = 260.L;
// Tuned with %VBus/RPM
// Talon SRX PID controller internally uses throttle units/encoder edges per 100ms
//
//           1023 throttle units per %VBus / (7 encoder pulses per motor rev *
//           26.9 motor revs per gear box output shaft rev [188.3] * 4 edges per quadrature encoder pulse[753.2]) /
//           (60 seconds per minute * 10 100ms per seconds)
const double KuUnits = 1023.L / (((uint16_t)(7L*26.9L + 0.5L)*4.L)/(60.L * 10.L));
//
// Tuned with Seconds;  Talon SRX PID controller internally uses milliseconds
//           1000 milliseconds per seconds
const double PuUnits = 1000.L;

// todo: Tank Turn Tread Drive Motors PID Tuning parameters go here (from AutoTuneTank run)
const double TankTurnKp = 2.5;
const double TankTurnKi = 0.01;
const double TankTurnKd = 0.; // some say use Kd = 0 for the secondary controller of a cascade (use PI not PID)

#endif

#ifdef ROBOT_CIMS
// no ConfigEncoderCodesPerRev()
// GetPosition() == GetEncPosition, GetSpeed == GetEncVel(); but updated faster than Enc versions
// max GetSpeed about 850 (2016 tank tread competition robot)
const uint16_t ENCODER_CODES_PER_REV = 0;
const double CONTROLLER_MIN = -850.L;
const double CONTROLLER_MAX = 850.L;
// Tuned with %VBus/encoder edges per 100ms
// Talon SRX PID controller internally uses throttle units/encoder edges per 100ms
//
//           1023 throttle units per %VBus
const double KuUnits = 1023.L;
//
// Tuned with Seconds;  Talon SRX PID controller internally uses milliseconds
//           1000 milliseconds per seconds
const double PuUnits = 1000.L;

// todo: Tank Turn Tread Drive Motors PID Tuning parameters go here (from AutoTuneTank run)
const double TankTurnKp = 2.5;
const double TankTurnKi = 0.01;
const double TankTurnKd = 0.; // some say use Kd = 0 for the secondary controller of a cascade (PI not PID)

// todo: Camera PID using this motor Tuning parameters go here (from AutoTuneCamera run)
const double CameraKp = 8.;
const double CameraKi = .2;
const double CameraKd = 2.;

#endif
}

namespace CAN_PORT
{
// TALONS
const int FRONT_LEFT = 0;
const int REAR_LEFT = 2;
const int FRONT_RIGHT = 1;
const int REAR_RIGHT = 3;
}

#endif /* CONSTANTS_H_ */
