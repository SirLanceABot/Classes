#ifndef ITG_3200_H_
#define ITG_3200_H_

#include <stdint.h>
#include "WPIlib.h"
#include <memory>

class ITG3200
{
public:
	ITG3200(I2C::Port port, uint8_t deviceAddress, int sampleRate, double initialAngle[], float sensitivity, double integrationRate = 0.01);
	~ITG3200();
	bool IsWorking();
	void Calibrate();
	void Get();
	void GetAngle(double angle[]);
	void GetAngleRate(double angleRate[]);
	float GetTemperature();
	void SetAngle(double angle[]);
	void SetSampleRate(uint8_t sampleRate);
	void GetOffset(double offset[]);
	double GetZ(); // returns the yaw (rotation)
	double GetX(); // returns the roll
	double GetY(); // returns the pitch
	void Test(); // not generally needed
	
	// I2C addresses
	static constexpr int ITG3200_I2C_ADDR = 0x68;
	static constexpr int ITG3200_I2C_ADDR_ALT = 0x69;
	
	// default sensitivity
	static constexpr float SENSITIVITY_SCALE_FACTOR = 14.375;

	// sample rate constants
	static constexpr int k8000Hz_256Hz = 0; // 256Hz low pass filter bandwidth,	8,000Hz Internal Sample Rate
	static constexpr int k1000Hz_188Hz = 1; // 188Hz low pass filter bandwidth,	1,000Hz Internal Sample Rate
	static constexpr int k1000Hz_98Hz  = 2; //  98Hz low pass filter bandwidth,	1,000Hz Internal Sample Rate
	static constexpr int k1000Hz_42Hz  = 3; //  42Hz low pass filter bandwidth,	1,000Hz Internal Sample Rate
	static constexpr int k1000Hz_20Hz  = 4; //  20Hz low pass filter bandwidth,	1,000Hz Internal Sample Rate
	static constexpr int k1000Hz_10Hz  = 5; //  10Hz low pass filter bandwidth,	1,000Hz Internal Sample Rate
	static constexpr int k1000Hz_5Hz   = 6; //   5Hz low pass filter bandwidth,	1,000Hz Internal Sample Rate

private:
	I2C* mGyro;
//	std::shared_ptr<I2C> mGyro;
	mutable priority_mutex m_mutex; // mutex to protect read/write access to process variable
	std::unique_ptr<Notifier> m_backgroundLoop;
	Timer* mTimer;
//	std::shared_ptr<Timer> mTimer;
	void GetRaw();
	float mSensitivity;
	double mAngle[3];
	uint8_t mBuffer[8];
	float mOffset[3];
	float mAngleRate[3];
	float mTemperature;
	float mRotation[3];
	bool mWorking;
	double mIntegrationRate;
	double mPrevTime;
	
	// id
	const int ITG3200_ID_REGISTER = 0x00;
	const int ITG3200_ID = 0x34;
	const int ITG3200_ID_BITS = 0x7e;

	// configuration registers
	const int ITG3200_SMPLRT_DIV = 0x15;
	const int ITG3200_DLPF_FS = 0x16;

	// interrupt registers
	const int ITG3200_INT_CFG = 0x17;
	const int ITG3200_INT_STATUS = 0x1A;

	// data registers (read only)
	const int ITG3200_TEMP_H = 0x1B;
	const int ITG3200_TEMP_L = 0x1C;
	const int ITG3200_XOUT_H = 0x1D;
	const int ITG3200_XOUT_L = 0x1E;
	const int ITG3200_YOUT_H = 0x1F;
	const int ITG3200_YOUT_L = 0x20;
	const int ITG3200_ZOUT_H = 0x21;
	const int ITG3200_ZOUT_L = 0x22;
	const int DATA_REG_SIZE = 8;

	// power management
	const int ITG3200_PWR_MGM = 0x3E;

	// useful values
	const int ITG3200_RESET = 0x80;
	const int ITG3200_SLEEP = 0x40;
	const int ITG3200_WAKEUP = 0x00;
	// FS=11 DLPF=000 => 11000 => 0x18 => 0b11'000
	const int ITG3200_FS_3 = 0x18;
};

#endif /* ITG_3200_H_ */
