package frc.robot;

import java.util.Timer;
import java.util.TimerTask;
import java.util.concurrent.atomic.AtomicInteger;

import edu.wpi.first.wpilibj.I2C;

public class LIDAR_Lite {
	/**
	 * Last parameter for integrationRate not specified.
	 * 
	 * <p>
	 * Using default Sampling Rate period 0.015 seconds
	 */
	public LIDAR_Lite(I2C.Port port, Address deviceAddress) {
		this(port, deviceAddress, 0.015 /* Default Sample Period in seconds */);
	}

	public LIDAR_Lite(I2C.Port port, Address deviceAddress, double samplePeriod) {
		System.out.println("[LIDAR-Lite] starting LIDAR class");

		mTimer = new Timer();
		mDeviceAvailable = false;
		mSamplePeriod = samplePeriod;

		mLIDAR = new I2C(port, deviceAddress.value); // define the device object on the I2C bus

		/***********
		 * reset probably not needed but this is an abundance of caution
		 **********/
		if (mLIDAR.write(Register.COMMAND.value, Command.RESET.value))
			System.out.printf("[LIDAR-Lite] write operation failed line %s\n", Id.__LINE__());
		else System.out.println("[LIDAR-Lite] Reset");
		edu.wpi.first.wpilibj.Timer.delay(1.0); // wait for the reset to finish - guess this should be long enough

		// read mode configuration register mostly just to see if the LIDAR-Lite is
		// working (I2C::AddressOnly() doesn't work)

		/********** Mode Configuration Register - default is 0x10 **********/
		byte modeConfiguration[] = new byte[Register.MODE_CONFIGURATION.count];
		byte modeConfigurationRegister[] = new byte[RegisterAddressLength];
		modeConfigurationRegister[RegisterAddressLength - 1] = (byte) Register.MODE_CONFIGURATION.value;

		if (mLIDAR.writeBulk(modeConfigurationRegister)) // request LIDAR-Lite configuration
		{
			System.out.printf("[LIDAR-Lite] writeBulk modeConfiguration failed line %s LIDAR-Lite NOT functional\n",
					Id.__LINE__());
		} else if (mLIDAR.readOnly(modeConfiguration, modeConfiguration.length)) // read the LIDAR-Lite configuration
		{
			System.out.printf("[LIDAR-Lite] readOnly modeConfiguration failed line %s LIDAR-Lite NOT functional\n",
					Id.__LINE__());
		} else {
			System.out.printf("[LIDAR-Lite] running with Mode Configuration %#2x\n", modeConfiguration[0]);

			mDeviceAvailable = true; // device seen so indicate that
		}

		if (IsWorking()) {
			System.out.printf("[LIDAR-Lite] working on port %s, address %s %#2x\n", port, deviceAddress, deviceAddress.value);

			//GetAcquisitionCount(); // as found 
			int maxAcquisitionCount = 0xff;  // new value to set
		 	if (mLIDAR.write(Register.ACQUISITION_COUNT.value, maxAcquisitionCount))
			 	System.out.printf("[LIDAR-Lite] write operation failed line %s\n", Id.__LINE__());
			GetAcquisitionCount(); // read it back

			StartPeriodic(mSamplePeriod);
		} else {
			System.out.printf("[LIDAR-Lite] NOT working on port %s, address %s %#2x\n", port, deviceAddress,
					deviceAddress.value);
		}
	}

	private void StartPeriodic(double period) {
		long loopTime = (long) (period * 1000. + .5);
		m_backgroundLoop = new UpdateLIDAR();
		mTimer.schedule(m_backgroundLoop, // TimerTask to run
				0, // initial delay is 0 - run immediately
				loopTime); // subsequent rate milliseconds
		System.out.println("[LIDAR-Lite] Background task running every " + loopTime + " milliseconds");
	}

	private void Stop() {
		if (m_backgroundLoop != null)
			m_backgroundLoop.cancel();
	}

	public boolean IsWorking() {
		return mDeviceAvailable;
	}

	private class UpdateLIDAR extends TimerTask {

		public void run() {
			Calculate(); // keep run() minimal - Get is where all the action is
		}
	}

	void Calculate() {
		byte distance[] = new byte[Register.DISTANCE_1_2.count]; // LIDAR-Lite command
		byte distanceRegister_1st[] = new byte[RegisterAddressLength]; // LIDAR-Lite command

		distanceRegister_1st[RegisterAddressLength - 1] = (byte) Register.DISTANCE_1_2.value; // hold the LIDAR-Lite
																								// returned values

		while (IsBusy()) {
			edu.wpi.first.wpilibj.Timer.delay(.005);
			if(IsBusy()) System.out.println("[LIDAR-Lite] Busy");
			else break;
		} // not expected to be busy here but check to make sure

		/*********** acquire distance **********/ // I2C::WriteBulk() also works
		if (mLIDAR.write(Register.COMMAND.value, Command.ACQUIRE_DC_CORRECT.value)) // initiate distance acquisition
																					// with DC stabilization
			System.out.printf("[LIDAR-Lite] write operation failed line %s\n", Id.__LINE__());

		while (IsBusy()) {
			edu.wpi.first.wpilibj.Timer.delay(.005);
			if(IsBusy()) System.out.println("[LIDAR-Lite] Busy acquiring");
			else break;
		} // should be busy briefly while acquiring distance

		/********** read distance **********/ // I2C::Read() does not work, I2C::Transaction() does not work
		if (mLIDAR.writeBulk(distanceRegister_1st)) // tell LIDAR-Lite we want to start reading at the 1st distance
													// register
			System.out.printf("[LIDAR-Lite] writeBulk distance failed line %s\n", Id.__LINE__());

		else if (mLIDAR.readOnly(distance, distance.length)) // read the 2 distance registers
			System.out.printf("[LIDAR-Lite] readOnly distance failed line %s\n", Id.__LINE__());

		else if (IsGoodReading()) { // have the 2 distance characters from the LIDAR-Lite registers so convert to
									// int for our use
			mDistance.set(((distance[0] & 0xFF) << 8) | (distance[1] & 0xFF)); // update shared memory with the new
																				// LIDAR-Lite value
			// how to turn 2 Java bytes into 1 Java essentially unsigned int (lidar 2 bytes
			// are llllllll rrrrrrrr)
			// left byte is converted to implied int conserving the sign that we don't want
			// as a sign and shifted to make room for the right byte.
			// llllllll => ssssssss ssssssss ssssssss llllllll => 00000000 00000000 00000000
			// llllllll => 00000000 00000000 llllllll 00000000
			// right byte is converted to implied int conserving the sign that we don't want
			// as a sign. (the sign comes from the first r as Java erroneously thinks)
			// rrrrrrrr => ssssssss ssssssss ssssssss rrrrrrrr => 00000000 00000000 00000000
			// rrrrrrrr
			// then OR (add) the 2 implied int pieces to make an implied int number.
			// 00000000 00000000 llllllll rrrrrrrr
		}

		else
			System.out.printf("[LIDAR-Lite] Not Good Reading Distance, status register:%#2x line %s\n", mStatus[0],
					Id.__LINE__());
		// mDistance is not changed if the distance read fails - old value is retained;
		// this may not be appropriate
	}

	public int GetDistance() // return distance from LIDAR
	{
		int returnDistance;

		if (IsWorking()) {
			returnDistance = mDistance.get();
		} else {
			returnDistance = -1;
		}
		return returnDistance;
	}

	public String toString() {
		if (IsWorking())
			return String.format("[LIDAR-Lite] distance = %d", GetDistance());
		else
			return String.format("[LIDAR-Lite] NOT working");
	}

	boolean GetStatus() // get the LIDAR-Lite status byte for others to interpret
	{
		byte statusRegister[] = new byte[RegisterAddressLength];
		statusRegister[RegisterAddressLength - 1] = (byte) Register.STATUS.value;

		/********** read status **********/
		if (mLIDAR.writeBulk(statusRegister)) // LIDAR-Lite command
		{
			System.out.printf("[LIDAR-Lite] writeBulk status failed! line %s\n", Id.__LINE__());
		} else {
			if (mLIDAR.readOnly(mStatus, mStatus.length)) // LIDAR-Lite command
			{
				System.out.printf("[LIDAR-Lite] readOnly status failed line %s\n", Id.__LINE__());
			} else {
				// System.out.printf ( "[LIDAR-Lite] Status Good Reading, status register:%#2x
				// line %s\n", mStatus[0], Id.__LINE__());
				return true;
			}
		}
		mStatus[0] = 0x01; // this didn't work out so fake bad status so it seems to be busy, unhealthy
		return false;
	}

	boolean GetAcquisitionCount() // get the LIDAR-Lite Acquisition Count
	{
		byte aquisitionCountRegister[] = new byte[RegisterAddressLength];
		aquisitionCountRegister[RegisterAddressLength - 1] = (byte) Register.ACQUISITION_COUNT.value;

		/********** read status **********/
		if (mLIDAR.writeBulk(aquisitionCountRegister)) // LIDAR-Lite command
		{
			System.out.printf("[LIDAR-Lite] writeBulk status failed! line %s\n", Id.__LINE__());
		} else {
			if (mLIDAR.readOnly(mAcquisitionCount, mAcquisitionCount.length)) // LIDAR-Lite command
			{
				System.out.printf("[LIDAR-Lite] readOnly status failed line %s\n", Id.__LINE__());
			} else {
				System.out.printf ( "[LIDAR-Lite] Aquisition Count %#2x\n", mAcquisitionCount[0]);
				return true;
			}
		}
		return false;
	}

	boolean IsBusy() // check LIDAR-Lite busy bit on; indicates if Lidar is busy (not ready for
						// another command)
	{
		if (GetStatus())
			return (mStatus[0] & StatusMask.BUSY.value & 0xFF) != 0; // bit 0 is LIDAR-Lite v2 busy bit
		else
			return true;
	}

	boolean IsHealthy() // check LIDAR-Lite healthy bit on; indicates if Lidar healthy
	{
		if (GetStatus())
			return (mStatus[0] & StatusMask.HEALTHY.value & 0xFF) != 0; // bit 5 is LIDAR-Lite v2 healthy bit
		else
			return false;
	}

	boolean IsGoodReading() // check LIDAR-Lite not-good bits all off and healthy bit on; indicates if
							// distance measurement was valid
	{
		if (GetStatus())
			return (mStatus[0] & StatusMask.NOT_GOOD_READING.value & 0xFF) == 0
					&& (mStatus[0] & StatusMask.HEALTHY.value & 0xFF) != 0; // true if no problem bits on and healthy
																			// bit on
		else
			return false;
	}

	/*
	 * Methods not implemented since they don't make much sense to do so for a LIDAR
	 * sensor: public int hashCode() public boolean equals(Object obj) copyFrom
	 * clone
	 */

	// register length for reads - all registers are the same length (1) for this
	// LIDAR so don't bother putting that in the enum for each different command
	private static final int RegisterAddressLength = 1;

	private static enum Register { // LIDAR internal control registers used in this program
		COMMAND(0x00, 0), // write the command only - nothing to read
		STATUS(0x01, 1), // read the single status byte
		ACQUISITION_COUNT(0x02, 1), // read/write the maximum acquisition count 
		DISTANCE_1_2(0x8f, 2), // using automatic sequence increment to read both (2) distance registers bytes
		MODE_CONFIGURATION(0x4b, 1); // read the single mode configuration byte
		public final int value;
		public final int count;

		private Register(int value, int count) {
			this.value = value; // register number
			this.count = count;
		} // bytes or registers to read in 1 read command
	};

	public static enum Address { // I2C addresses
		DEFAULT(0x62); // default I2C bus address for the LIDAR Lite v2
		public final int value;

		private Address(int value) {
			this.value = value; // need this if "casting" Address to int for "external" usage and array size
		}
	}

	private static enum Command { // command used in the COMMAND control register 0x00; we only use in this
									// program 1 of the several commands allowed
		RESET(0x00),
		ACQUIRE_DC_CORRECT(0x04);
		public final int value;

		private Command(int value) {
			this.value = value; // need this if "casting" Address to int for "external" usage and array size
		};
	}

	private static enum StatusMask { // LIDAR status register bits masks
		HEALTHY(0x20), BUSY(0x01), NOT_GOOD_READING(0x51);
		public final int value;

		private StatusMask(int value) {
			this.value = value; // need this if "casting" Address to int for "external" usage and array size
		}
	}

	// Global Variables
	private final I2C mLIDAR;
	private final Timer mTimer;
	private final double mSamplePeriod;
	private TimerTask m_backgroundLoop;
	private boolean mDeviceAvailable; // indicate if the Lidar is seen on the I2C bus; checked only once at startup
	private byte[] mStatus = new byte[Register.STATUS.count]; // status from the Lidar
	private byte[] mAcquisitionCount = new byte[Register.ACQUISITION_COUNT.count]; // acquisition count from the Lidar
	private AtomicInteger mDistance = new AtomicInteger(-2); // distance from Lidar [cm]
}
/*
 * Writing data to the LIDAR
 * 	Write to the device
 * 		Byte 1 is the register address to be written into
 * 		Byte 2 is the data to write into the register
 * 
 * 
 * Reading data from the LIDAR
 * 	Write to the device
 * 		Byte 1 is the register address to be read
 * 	Read from the device the number of registers desired
 * 
 * Note that the device address is written before each read/write function

Command Register
Write 0x04 to Register 0x00: Take acquisition & correlation processing with DC correction

0x01 - Mode/Status (control_reg[1]:)

mode/status register 0x01 (1)
Bit	Function	Notes
Bit 7	Eye Safe	This bit will go high if eye-safety protection has been activated
Bit 6	Error Detection	Process error detected / measurement invalid
Bit 5	Health	"1" if good, "0" if bad
Bit 4	Secondary return	Secondary return detected above correlation noise floor threshold
Bit 3	Signal not valid	Indicates that the signal correlation peak is equal to or below correlation record noise threshold
Bit 2	Sig overflow flag	Overflow detected in correlation process associated with a signal acquisition
Bit 1	Ref overflow flag	Overflow detected in correlation process associated with a reference acquisition
Bit 0	Ready Status	"0" is ready for new command, "1" is busy with acquisition
Health status indicates that the preamp is operating properly, transmit power is active and a reference pulse has been processed and has been stored.

Acquisition Count register 0x02 (2)
Eight bits for Maximum Acquisition Count
range 0x00 to 0xff [0 to 255]
default 0x80

mode configuration register 0x4b (75)
Bit	Function	Notes
Bit 7	Disable DC Correction	Skips DC stabilization. Used to increase repetition rate
Bit 6	NOT USED
Bit 5	NOT USED
Bit 4	Select Reference/Signal	Selects Reference or Signal Records when operating from control register 0x40
Bit 3	NOT USED
Bit 2	Select Max Range	"1" selects the longer distance; "0" selects the shorter distance
Bit 1	Select Range Criteria	"1" selects return data based on distance; "0" selects strongest return, regardless of distance
Bit 0	Select Second Return	Controls echo processing selection :"1" switches to alternative return; "0" Selects data associated with detection criteria
 */
