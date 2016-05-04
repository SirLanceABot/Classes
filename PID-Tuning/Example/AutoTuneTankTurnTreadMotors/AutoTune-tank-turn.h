#ifndef AUTOTUNEEXAMPLE_H_
#define AUTOTUNEEXAMPLE_H_

class CANTalon;

int TuneMain(CANTalon* mDriveMotor1, CANTalon* mDriveMotor2, CANTalon* mDriveMotor3, CANTalon* mDriveMotor4, bool TankTurn);

#endif
