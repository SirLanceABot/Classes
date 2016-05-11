#ifndef CANTalon4237_H
#define CANTalon4237_H

class CANTalon;

class CANTalon4237
{
public:
	enum SensorType {kNoSensor, kSwitchNormallyOpen, kQuadEncoder, kMagneticEncoder, kSwitchAndQuadEncoder, kSoftLimitAndQuadEncoder};
//	CANTalon4237(int, SensorType);
	CANTalon4237(int, SensorType, int = 0, int = 0);
	~CANTalon4237();
	void Set(float);
	void ResetEncoder();
	void SetFollower(int);
	void SetMaster();
	int GetEncoder();
	void ReverseEncoderDirection();

private:
	CANTalon* mCANTalon;
	SensorType mSensorType;
};

#endif
