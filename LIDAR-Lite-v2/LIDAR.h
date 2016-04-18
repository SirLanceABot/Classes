#ifndef SRC_LIDAR_H_
#define SRC_LIDAR_H_

#include <wpilib.h>

class Lidar
/*
 * Written by Sir Lance-A-Bot / Team FRC 4237
 *
 * FRC 2016 Program to obtain distance from robot to an object with a PulsedLight LIDAR Lite v2 connected to an I2C bus
 */
{
public:

	enum Address {ADDRESS_DEFAULT=0x62}; // default I2C bus address for the LIDAR Lite v2
	Lidar(I2C::Port port, uint8_t deviceAddress, float samplePeriod = 0.02);
	unsigned int GetDistance(); // distance from Lidar
	bool IsWorking(); // indicate if the Lidar is seen on the I2C bus
	~Lidar();  // object destructor
	void Calculate(); // start of the background loop program

private:

	enum Register {COMMAND=0x00, STATUS=0x01, DISTANCE_1_2=0x8f, MODE_CONFIGURATION=0x4b};  // defined by Lidar
	enum Command {ACQUIRE_DC_CORRECT=0x04};  // defined by Lidar
	enum NumberOfRegistersToRead {READ_1_REGISTER=0x01, READ_2_REGISTERS=0x02};  // defined by Lidar
	enum NumberOfRegistersToWrite {WRITE_1_REGISTER=0x01};  // defined by Lidar
	enum StatusBits {HEALTHY=0x20, BUSY=0x01, NOT_GOOD_READING=0x51}; // defined by Lidar
	unsigned char mStatus[Lidar::READ_1_REGISTER]; // status from the Lidar
	I2C* mI2CBus; // define I2C bus
	bool mDeviceAvailable; // indicate if the Lidar is seen on the I2C bus
	bool GetStatus(); // get the Lidar status
	bool IsBusy();  // indicates if Lidar is busy (not ready for another command)
	bool IsHealthy(); // indicates if Lidar healthy
	bool IsGoodReading(); // indicates if distance measurement was valid
	pthread_t mThread;  // background acquire distance thread
	float m_period;     // time period for background loop in seconds
	mutable priority_mutex m_mutex; // mutex to protect read/write access to distance
	unsigned int mDistance; // distance from Lidar
	double mLidarSampleTime; 	// minimum loop time for the Lidar distance sampling background thread
	;
};

#endif /* SRC_LIDAR_H_ */
