#ifndef CameraTargetingPID_H_
#define CameraTargetingPID_H_

class CANTalon;

int TargetMain(CANTalon * mFrontLeftMotor, CANTalon * mFrontRightMotor, CANTalon * mRearLeftMotor, CANTalon * mRearRightMotor);

#endif
