Examples assuming NI roboRIO, FRC WPILib, and CTRE CANTalon usage.

These examples are a little bit of a mishmash of stuff but they get the tuning and bal shooting targeting done plus some driving straight exercises.

A weakness in the codes as is are none check for PID secondary controller saturation.  The primary PID controller could keep calling for more speed when the drive motors are already maxed out.
There are controller limits but they are (good) guesses of the limits of the motors but aren't guaranteed and there is no check that these are the true limits.

There is no automatic connection of PID tuning parameters to subsequent driving - must update constants.h files by hand.

Warning about CTRE CAN Talon motor reversals:

The CAN Talon functions have 3 different motor reversal functions to support (easily?) a pair of motors being mirrored.

```C++
// right motor is mirror of left so invert its actions since it runs backwards of the left motor

mFrontRightMotor->SetInverted(true);
 // invert power so + power goes forward for both sides; not used for PID control of mirrored motors - must be false; only for %VBus then must be true

mFrontRightMotor->SetSensorDirection(true);
 // invert encoder to match left; true reverses GetPosition & GetSpeed but not GetEncPosition nor GetEncVel
 // if SetInverted(true) for %VBus or SetClosedLoopOutputDirection(true) for closed loop PID control then probably want to set this true also.
 
mFrontRightMotor->SetClosedLoopOutputDirection(true);
 // reverses the power to the motor in PID controller mode similarly to SetInverted(true) for %VBus
 // value ignored for %VBus and only checked if in closed loop control
``` 
So if mixing do-it-yourself driving such as with the PID controllers and then without PID control and with the WPILib drive functions such as ArcadeDrive
be be aware of the need to manage the SetInverted.  ArcadeDrive does its own +- power setting so SetInverted(true) messes up that built-in, behind-the-scenes scheme.

The SetInverted setting could also depend on setting the follower mode.
Again ArcadeDrive, if given 4 motors, already handles the follower mode its own way and using the CAN Talon follower mode would mess up that scheme.

So setting follower mode also must the managed well.  Extra motors would follow the PID controlled motors.
Extra motors would follow the primary motors if using ArcadeDrive with 2 motors but must use all 4 motors as %VBus if using ArcadeDrive with 4 motors.

**AutoTuneTank**
	**Test**
		tunes the tank tread PID controller
		sets front motors %VBus with rear motor followers for Talon motor controllers and then leaves them that way
	**OperatorControl**
		drives straight using PID speed control
		sets front motors PID speed control with rear motor followers for Talon motor controllers and then leaves them that way

**AutoTuneCamera**
	**Test**
		tunes camera aiming at target
		sets front motors PID control with rear followers and then leaves all motors %VBus
		
**CameraTargeting**
	**RobotInit**
		sets front motors PID speed control with rear motor followers
	**Test**
		uses camera to aim target
		then leaves all motors %VBus
	**OperatorControl**
		drives straight using PID controllers and followers as set in RobotInit
