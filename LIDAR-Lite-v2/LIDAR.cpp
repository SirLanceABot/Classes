/*
 * Written by Sir Lance-A-Bot / Team FRC 4237
 *
 * FRC 2016 Program to obtain distance from robot to an object with a PulsedLight LIDAR-Lite v2 connected to an I2C bus
 *
 * Note that for the LIDAR-Lite v2, WPILib I2CRead() and Transaction() don't work probably because the I2C start/end
 * transaction flags are not what the LIDAR-Lite requires.
 *
 * Note that on the roboRIO Expansion Chassis, at least, WPILib I2C AddressOnly() always returns false (device present
 * even if there is no device connected.
 *
 * This program starts a "background" thread always polling the LIDAR-Lite for the current distance.
 * The user may request the current distance at any time.
 */
#include "LIDAR.h"

// function to start the background thread
// takes the calling objects "this" as the parameter so that object can be used for the rest of the references
void * CallCalculate(void * controller)
{ // now in the background thread
	if ( pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL) ) // suppress canceling of this thread to get started
		printf("[LIDAR-Lite] %p initial disable cancel state failed.  line %d\n", controller, __LINE__);
	Lidar *control = (Lidar*) controller; // refer back to calling object
	control->Calculate(); // continue on with this thread using the calling object
	pthread_exit(NULL); // thread exit
}

Lidar::Lidar(I2C::Port port, uint8_t deviceAddress, float samplePeriod)
	: mDeviceAvailable(false), mDistance(-1), mLidarSampleTime(samplePeriod)
{
	printf("[LIDAR-Lite] %p Compiled file:%s, date:%s, time:%s, function:%s, function:%s, printed at line:%d\n",
			this, __FILE__, __DATE__, __TIME__, __FUNCTION__, __PRETTY_FUNCTION__, __LINE__);

	if(port == I2C::Port::kMXP) printf("[LIDAR-Lite] %p Expansion Board I2C Bus (port:%d), address:%#.2x\n", this, port, deviceAddress);
	else
	if(port == I2C::Port::kOnboard) printf("[LIDAR-Lite] %p On Board I2C Bus (port:%d), address:%#.2x\n", this, port, deviceAddress);
	else
	printf("[LIDAR-Lite] %p Unknown I2C bus (port:%d), address:%#.2x\n", this, port, deviceAddress);

	mI2CBus = new I2C(port, deviceAddress); // define the device object on the I2C bus

	// read mode configuration register mostly just to see if the LIDAR-Lite is working (I2C::AddressOnly() doesn't work)

	/**********Mode Configuration Register - default is 0x10 **********/
	unsigned char modeConfiguration[Lidar::READ_1_REGISTER];
	unsigned char modeConfigurationRegister[Lidar::WRITE_1_REGISTER];
	modeConfigurationRegister[Lidar::WRITE_1_REGISTER-1] = Lidar::MODE_CONFIGURATION;

	if ( mI2CBus->WriteBulk(modeConfigurationRegister, Lidar::WRITE_1_REGISTER)) // request LIDAR-Lite configuration
		{
		printf ( "[LIDAR-Lite] %p WriteBulk modeConfiguration failed line %d LIDAR-Lite NOT functional\n", this, __LINE__);
		}
	else
	if ( mI2CBus->ReadOnly(Lidar::READ_1_REGISTER, modeConfiguration)) // read the LIDAR-Lite configuration
		{
		printf ( "[LIDAR-Lite] %p ReadOnly modeConfiguration failed line %d LIDAR-Lite NOT functional\n", this, __LINE__);
		}
	else
		{
		printf("[LIDAR-Lite] %p running with Mode Configuration:%#.2x\n", this, modeConfiguration[0]);

		mDeviceAvailable = true; // device seen so indicate that

		if ( pthread_create(&mThread, NULL, CallCalculate, this) ) // start the background thread to get distance periodically
					printf("[LIDAR-Lite] %p thread create failed. line %d\n", this, __LINE__);
		}
}

Lidar::~Lidar()
{
	if ( IsWorking() )
		{
		// request thread that was started to be terminated
		if ( pthread_cancel (mThread) )		// send cancel request to the background thread
			printf("[LIDAR-Lite] %p thread cancel failed. line %d\n", this, __LINE__);

		if ( pthread_join (mThread, NULL) )	// wait for thread to end
			printf("[LIDAR-Lite] %p thread join failed. line %d\n", this, __LINE__);

		printf("[LIDAR-Lite] %p distance background thread terminated\n", this);
		}
}

void Lidar::Calculate()
{
	Timer mTimer; // timer used to pace the loop
	float loopStartTime; // start time of each loop
	unsigned char distance[Lidar::READ_2_REGISTERS]; // LIDAR-Lite command
	unsigned char distanceRegister_1st[Lidar::WRITE_1_REGISTER]; // LIDAR-Lite command
	distanceRegister_1st[Lidar::WRITE_1_REGISTER-1] = Lidar::DISTANCE_1_2; // hold the LIDAR-Lite returned values

	printf("[LIDAR-Lite] %p Distance Background Thread Running\n", this);

	mTimer.Start(); // start the timer

	while(true) // running in the background forever until there is a cancel signal
	{
	loopStartTime = mTimer.Get(); // the time this loop started

	if ( pthread_setcancelstate(PTHREAD_CANCEL_ENABLE, NULL) ) // allow canceling here before the loop does anything to the LIDAR-Lite
		printf("[LIDAR-Lite] %p loop enable cancel state failed. line %d\n", this, __LINE__);

	pthread_testcancel();  // set this location as a cancel point so if cancel happens, there's no coming back to here to continue on

	if ( pthread_setcancelstate(PTHREAD_CANCEL_DISABLE, NULL) ) // must not have been a request to cancel; suppress canceling for rest of the loop
		printf("[LIDAR-Lite] %p loop disable cancel state failed. line %d\n", this, __LINE__);

	while (Lidar::IsBusy()) {Wait(.005);} // not expected to be busy here but check to make sure

	/***********acquire distance**********/		//	I2C::WriteBulk() also works
	if ( mI2CBus->Write(Lidar::COMMAND, Lidar::ACQUIRE_DC_CORRECT) ) // initiate distance acquisition with DC stabilization
		printf ( "[LIDAR-Lite] %p Write operation failed line %d\n", this, __LINE__ );

	while (Lidar::IsBusy()) {Wait(.005);} // should be busy briefly while acquiring distance

	/**********read distance**********/     // I2C::Read() does not work, I2C::Transaction() does not work
	if      ( mI2CBus->WriteBulk(distanceRegister_1st, Lidar::WRITE_1_REGISTER) ) // tell LIDAR-Lite we want to start reading at the 1st distance register
		printf ( "[LIDAR-Lite] %p WriteBulk distance failed line %d\n", this, __LINE__ );

	else if ( mI2CBus->ReadOnly(Lidar::READ_2_REGISTERS, distance) ) // read the 2 distance registers
		printf ( "[LIDAR-Lite] %p ReadOnly distance failed line %d\n", this, __LINE__ );

	else if ( Lidar::IsGoodReading() )
	{ // have the 2 distance characters from the LIDAR-Lite registers so convert to unsigned int for our use
		unsigned int dist = (unsigned int)(distance[0]<<8) + (unsigned int)(distance[1]);
		std::lock_guard<priority_mutex> sync(m_mutex); // protect the shared memory from others' access while it's being updated here
		mDistance = dist; // update shared memory with the new LIDAR-Lite value
	}

	else printf ( "[LIDAR-Lite] %p Not Good Reading, status register:%#.2x line %d\n", this, mStatus[0], __LINE__);

	// pace the loop time - aim for fixed sampling rate
	double waitTime = mLidarSampleTime - (mTimer.Get() - loopStartTime); // how much time left to kill in this loop
	if (waitTime > 0.0) Wait(waitTime); // wait that much time
	else printf("[LIDAR-Lite] %p Slow Sample Distance Loop By: %f Seconds line %d\n", this, -waitTime, __LINE__); // ran over the requested time limit
	}
}

unsigned int Lidar::GetDistance()
{
	unsigned int returnDistance;

	if ( IsWorking() )
		{
		std::lock_guard<priority_mutex> sync(m_mutex); // protect shared memory from background update while it's being read here by user
		returnDistance = mDistance;
		}
	else returnDistance = -1;

	return returnDistance;
}

bool Lidar::IsWorking() // return what was determined when LIDAR-Lite was first started
{
	return mDeviceAvailable;
}

bool Lidar::GetStatus() // get the LIDAR-Lite status byte for others to interpret
	{
		unsigned char statusRegister[Lidar::WRITE_1_REGISTER];
		statusRegister[Lidar::WRITE_1_REGISTER-1] = Lidar::STATUS;

		/**********read status**********/
		if ( mI2CBus->WriteBulk(statusRegister, Lidar::WRITE_1_REGISTER)) // LIDAR-Lite command
			{printf ( "[LIDAR-Lite] %p WriteBulk status failed! line %d\n", this, __LINE__ );}
		else
		if ( mI2CBus->ReadOnly(Lidar::READ_1_REGISTER, mStatus) ) // LIDAR-Lite command
			{printf ( "[LIDAR-Lite] %p ReadOnly status failed line %d\n", this, __LINE__ );}
		else
			{
			return true;
			}

		mStatus[0] = 0x01; // this didn't work out so fake bad status so it seems to be busy, unhealthy
		return false;
	}

bool Lidar::IsBusy() // check LIDAR-Lite busy bit on
	{
		if ( GetStatus() )	return mStatus[0] & (unsigned char)Lidar::BUSY; // bit 0 is LIDAR-Lite v2 busy bit
		return true;
	}

bool Lidar::IsHealthy() // check LIDAR-Lite healthy bit on
	{
		if ( GetStatus() )	return mStatus[0] & (unsigned char)Lidar::HEALTHY; // bit 5 is LIDAR-Lite v2 healthy bit
		return false;
	}

bool Lidar::IsGoodReading() // check LIDAR-Lite not-good bits all off and healthy bit on
	{
		if ( GetStatus() )	return (0 == (mStatus[0] & (unsigned char)Lidar::NOT_GOOD_READING)) && (mStatus[0] & (unsigned char)Lidar::HEALTHY); // true if no problem bits on and healthy bit on
		return false;
	}
/*
Command Register
Write 0x04 to Register 0x00: Take acquisition & correlation processing with DC correction

0x01 - Mode/Status (control_reg[1]:)

mode/status register 0x01 (1)
Bit	Function	Notes
Bit 7	Eye Safe	This bit will go high if eye-safety protection has been activated
Bit 6	Error Detection	Process error detected / measurement invalid
Bit 5	Health	“1” if good, “0” if bad
Bit 4	Secondary return	Secondary return detected above correlation noise floor threshold
Bit 3	Signal not valid	Indicates that the signal correlation peak is equal to or below correlation record noise threshold
Bit 2	Sig overflow flag	Overflow detected in correlation process associated with a signal acquisition
Bit 1	Ref overflow flag	Overflow detected in correlation process associated with a reference acquisition
Bit 0	Ready Status	“0” is ready for new command, “1” is busy with acquisition
Health status indicates that the preamp is operating properly, transmit power is active and a reference pulse has been processed and has been stored.

mode configuration register 0x4b (75)
Bit	Function	Notes
Bit 7	Disable DC Correction	Skips DC stabilization. Used to increase repetition rate
Bit 6	NOT USED
Bit 5	NOT USED
Bit 4	Select Reference/Signal	Selects Reference or Signal Records when operating from control register 0x40
Bit 3	NOT USED
Bit 2	Select Max Range	“1” selects the longer distance; “0” selects the shorter distance
Bit 1	Select Range Criteria	“1” selects return data based on distance; “0” selects strongest return, regardless of distance
Bit 0	Select Second Return	Controls echo processing selection :”1” switches to alternative return; “0” Selects data associated with detection criteria
*/
