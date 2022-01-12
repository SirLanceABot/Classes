package frc.robot;

import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;

public class AMSColorSensor
{

	// Global Variables - Fields
	private final I2C AMSColorSensorI2C;
	private final Timer timer;
	// private final double samplePeriod;
	private TimerTask m_backgroundLoop;
	private boolean deviceAvailable; // indicate if the Lidar is seen on the I2C bus; checked only once at startup
	//private byte[] mStatus = new byte[Register.STATUS.count]; // status from the Lidar
	int c, r, g, b;
	boolean isGoodReading;

	/**
	 * Last parameter for integrationRate not specified.
	 * 
	 * <p>Using default Sampling Rate period 0.015 seconds
	 */
	public AMSColorSensor(I2C.Port port, Address deviceAddress)
	{
        this(port, deviceAddress, 0.015 /* Default Sample Period in seconds*/);
        System.out.println(this.getClass().getName() + ": Started Constructing");
        System.out.println(this.getClass().getName() + ": Finished Constructing");
	}

	public AMSColorSensor(I2C.Port port, Address deviceAddress, double samplePeriod)
	{
		System.out.println("[ColorSensor] starting ColorSensor class");

		c = -1;
		r = -1;
		g = -1;
		b = -1;
		isGoodReading = false;

		this.timer = new Timer();
		this.deviceAvailable = false;
		// this.samplePeriod = samplePeriod;

		AMSColorSensorI2C = new I2C(port, deviceAddress.value); // define the device object on the I2C bus

		// Read device ID and check that it's expected model
		if( readAllDataRegisters()[Register.DEVICE_ID.bVal] != 0x60)
		{
			System.out.printf ( "[ColorSensor] read incorrect device id at line %s\n", Id.__LINE__());
		}
		else
		{	
			// Turn off sensing before changing registers because maybe it's a good idea but not sure it's necessary
			if( writeADataRegister(Register.ENABLE.bVal, (byte)0))
			{
				System.out.printf ( "[ColorSensor] writeBulk command register failed line %s ColorSensor NOT functional\n", Id.__LINE__());
			}
			else
			{
				deviceAvailable = true;

				// set all the registers mostly to default values and turn on sensor
				int SetDataRegisterValue;
				for (byte i = 15; i >= 0; i--)
				{
					try {
						Thread.sleep(10);
					} catch (InterruptedException e) {
						e.printStackTrace();
					}

					if(i == 0) SetDataRegisterValue = Enable.AEN.bVal | Enable.PON.bVal; // enable color sensor and power on
					else if(i == 1) SetDataRegisterValue = 0xFB; // ATIME: not many photons seen at 0xFF speed and AGAIN of 1x
					else if(i == 2 || i == 3) SetDataRegisterValue = 0xFF; // what we found to be the power on state
					else if(i == 15) SetDataRegisterValue = 0x23; // AGAIN: 0x20 = 1x; 0x21 = 4x; 0x22 = 16x; 0x23 = 60x
					else SetDataRegisterValue = 0x0; // what we found to be the power on state for most registers

					if( writeADataRegister(i, (byte)SetDataRegisterValue))
					{
						System.out.printf( "[ColorSensor] writeBulk command register failed line %s ColorSensor NOT functional\n", Id.__LINE__());
						deviceAvailable = false;
					}
				}
				System.out.print( "[ColorSensor] ");
				printAllDataRegisters();
			}
		}

		if (isWorking())
		{
			System.out.printf("[ColorSensor] working on port %s, address %s %#2x\n", port, deviceAddress, deviceAddress.value);
			startPeriodic(samplePeriod);
		}
		else
		{
			var colorNotWorking = String.format("[ColorSensor] NOT working on port %s, address %s %#2x\n", port, deviceAddress, deviceAddress.value);
			DriverStation.reportError(colorNotWorking, false);		}
	}

	private boolean writeADataRegister(byte register, byte data)
	{
		byte WriteARegister[] = new byte[2]; // CommandRegisterLength + DataRegisterLength

		WriteARegister[0] = (byte) (Command.COMMAND_SELECT.value | register);
		WriteARegister[1] = data;

		if ( AMSColorSensorI2C.writeBulk(WriteARegister))
		{
			System.out.printf ( "[ColorSensor] writeADataRegister writeBulk command failed line %s ColorSensor NOT functional\n", Id.__LINE__());
			return true;
		}
		else
		{
			System.out.format("[ColorSensor] set register %#2x to %#2x\n", register, WriteARegister[1]);
			return false;
		}
	}

	private byte[] readAllDataRegisters()
	{
		byte[] dataRegisterValues = new byte[0x1E];

		int CommandRegister = Command.COMMAND_SELECT.value | 0; // start reading at register 0

		if ( AMSColorSensorI2C.read(CommandRegister, dataRegisterValues.length, dataRegisterValues))
		{
			System.out.printf ( "[ColorSensor] readAllDataRegisters read command failed line %s ColorSensor NOT functional\n", Id.__LINE__());
			return dataRegisterValues;
		}
		else return dataRegisterValues;
	}

	private void printAllDataRegisters()
	{
		byte CommandRegister[] = new byte[1]; // CommandRegisterLength

		CommandRegister[0] = (byte) (Command.COMMAND_SELECT.value | Command.AUTO_INCREMENT_PROTOCOL.value);

		if ( AMSColorSensorI2C.writeBulk(CommandRegister)) // request ColorSensor configuration
		{
			System.out.printf ( "[ColorSensor] printAllDataRegisters writeBulk command register failed line %s ColorSensor NOT functional\n", Id.__LINE__());
		}
		else
		{
			byte allRegisters[] = new byte[0x1E]; // registers 0x00 to 0x1D inclusive
			if ( AMSColorSensorI2C.readOnly(allRegisters, allRegisters.length)) // read the ColorSensor configuration
			{
				System.out.printf ( "[ColorSensor] readOnly all registers failed line %s ColorSensor NOT functional\n", Id.__LINE__());
			}
			else
			{
				for (int i=0; i < allRegisters.length; i++)
				{
					System.out.printf("%2x ", allRegisters[i]);
				}
				System.out.println();
			}
		}
	}

	private void startPeriodic(double period) {
		long loopTime = (long)(period*1000. + .5);
		m_backgroundLoop = new updateColorSensor();
		timer.schedule(m_backgroundLoop, // TimerTask to run
				0,        // initial delay is 0 - run immediately
				loopTime);  // subsequent rate milliseconds
		System.out.println("[ColorSensor] Background task running every " + loopTime + " milliseconds");
	}

	public void stop() {
		if (m_backgroundLoop != null) m_backgroundLoop.cancel();
	}

	public boolean isWorking()
	{
		return deviceAvailable;
	}

	private class updateColorSensor extends TimerTask {

		public void run() {
			calculate(); // keep run() minimal - Get is where all the action is
		}
	}

	void calculate()
	{
		if (isWorking())
		{
			boolean rgbcCompleted = false; // assume reading will be bad
			// try 3 times to get a good reading
			for (int i = 1; i <=3; i++)
			{
				byte[] allRegisters = readAllDataRegisters();
				byte Status = allRegisters[0x13];
				if ( (Status & StatusMask.RGBC_CYCLE_COMPLETED.value & 0xFF) != 0)
				{ // found masked bits so it's good
					int C = (allRegisters[0x14] & 0xFF) | ((allRegisters[0x15] & 0xFF) << 8);
					int R = (allRegisters[0x16] & 0xFF) | ((allRegisters[0x17] & 0xFF) << 8);
					int G = (allRegisters[0x18] & 0xFF) | ((allRegisters[0x19] & 0xFF) << 8);
					int B = (allRegisters[0x1A] & 0xFF) | ((allRegisters[0x1B] & 0xFF) << 8);
					set(C, R, G, B, true); // save these data
					rgbcCompleted = true;
					//System.out.format("Status %#2x, CRGB: %d %d %d %d\n", Status, C, R, G, B);
					break; // good reading completed so move on
				}
				else
				{ // didn't find masked bits so it's bad and try again in awhile
					try {
						Thread.sleep(4); // sleep a little before trying again
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
			}

			// done reading and check if last one was good or all bad
			if(!rgbcCompleted)
			{ // tried to read sensor a few times and all failed so mark the data for this iteration as bad
				set(-1, -1, -1, -1, false);	//bad
				System.out.println("[ColorSensor] failed reading");
			}
		}
		// have the 2 characters from the ColorSensor registers so convert to int for our use
		//		.set( ((distance[0] & 0xFF) <<8) | (distance[1] & 0xFF) ); // update shared memory with the new ColorSensor value
		// how to turn 2 Java bytes into 1 Java essentially unsigned int (lidar 2 bytes are llllllll rrrrrrrr)
		// left byte is converted to implied int conserving the sign that we don't want as a sign and shifted to make room for the right byte.
		// llllllll => ssssssss ssssssss ssssssss llllllll => 00000000 00000000 00000000 llllllll => 00000000 00000000 llllllll 00000000
		// right byte is converted to implied int conserving the sign that we don't want as a sign. (the sign comes from the first r as Java erroneously thinks)
		// rrrrrrrr => ssssssss ssssssss ssssssss rrrrrrrr => 00000000 00000000 00000000 rrrrrrrr 
		//				// then OR (add) the 2 implied int pieces to make an implied int number. 00000000 00000000 llllllll rrrrrrrr
	}

	public synchronized void get(Colors crgb)
	{
		crgb.C = c;
		crgb.R = r;
		crgb.G = g;
		crgb.B = b;
		crgb.isGoodReading = isGoodReading;
	}

	public synchronized void set(int aC, int aR, int aG, int aB, boolean aisGoodReading)
	{
		c = aC;
		r = aR;
		g = aG;
		b = aB;
		isGoodReading = aisGoodReading;
	}

	public synchronized String toString()
	{
		if (isWorking())
			return String.format("[ColorSensor] CRGB %d %d %d %d", c, r, g, b);
		else
			return String.format("[ColorSensor] NOT working");
	}

	/*
		Methods not implemented since they don't make much sense to do so for a Color Sensor:
			public int hashCode()
			public boolean equals(Object obj)
			copyFrom
			clone
	 */

	// register length for reads - all registers are the same length (1) so don't bother putting that in the enum for each different command
	// private static final int registerAddressLength = 1;
	// private static final int commandRegisterLength = 1;

	enum Register
	{
		ENABLE(0x00),
		ATIME(0x01),
		REGISTER2(0x02), // unknown register (omitted from documentation) Is this actually PTIME?
		WTIME(0x03),     // Wait time = if AMS_COLOR_ENABLE_WEN is asserted
		AILT(0x04),
		AIHT(0x06),
		PERS(0x0C),
		CONFIGURATION(0x0D),
		PPLUSE(0x0E),    // proximity sensor pulse count config
		CONTROL(0x0F),
		DEVICE_ID(0x12),
		STATUS(0x13),
		ALPHA(0x14),     // clear color value: 0x14 = low byte, 0x15 = high byte
		RED(0x16),       // red color value: 0x16 = low byte, 0x17 = high byte
		GREEN(0x18),     // etc.
		BLUE(0x1A),
		PDATA(0x1C),     // proximity value: short: low=0x1C high=0x1D

		READ_WINDOW_FIRST(WTIME.bVal), // may as well...
		READ_WINDOW_LAST(BLUE.bVal +1);

		public final byte bVal;
		Register(int i) { this.bVal = (byte) i; }
	}

	public static enum Address { // I2C addresses
		DEFAULT(0x39); // default I2C bus address for the LIDAR Lite v2
		public final int value;
		private Address(int value){
			this.value = value; // need this if "casting" Address to int for "external" usage and array size
		}
	}

	private static enum Command { // command used in the AMS Color Sensor COMMAND register;
		COMMAND_SELECT (0x80), // must use this and one of the following - OR them together
		REPEATED_BYTE_PROTOCOL (0x00),
		AUTO_INCREMENT_PROTOCOL (0x20),
		RESERVED (0x40),
		SPECIAL_FUNCTION (0x60);
		public final int value;
		private Command(int value){
			this.value = value; // need this if "casting" Address to int for "external" usage and array size
		};
	}

	private static enum Enable
	{
		RESERVED7(0x80), /* reserved, write as zero */
		RESERVED6(0x40), /* reserved, write as zero */
		PIEN(0x20), /* Proximity interrupt enable ('reserved' on the TCS3472) */
		AIEN(0x10), /* RGBC Interrupt Enable */
		WEN(0x08),  /* Wait enable - Writing 1 activates the wait timer */
		PEN(0x04),  /* Proximity enable ('reserved' on the TCS3472) */
		AEN(0x02),  /* RGBC Enable - Writing 1 actives the ADC, 0 disables it */
		PON(0x01),  /* Power on - Writing 1 activates the internal oscillator, 0 disables it */
		OFF(0x00);  /* Nothing on */

		// public byte bitOr(Enable him) { return (byte)(this.bVal | him.bVal); }
		// public byte bitOr(byte him)   { return (byte)(this.bVal | him); }

		public final byte bVal;
		Enable(int i) { this.bVal = (byte) i; }
	}

	private static enum StatusMask // Color Sensor status register bits masks
	{	
		RGBC_CYCLE_COMPLETED(0x01);
		public final int value;
		private StatusMask(int value){
			this.value = value;
		}
	}

	public static class Constants
	{
		// public static final I2C.Port PORT = I2C.Port.kOnboard; // select roboRIO I2C port
		public static final I2C.Port MXP = I2C.Port.kMXP; // select roboRIO I2C port
		public static final I2C.Port ONBOARD = I2C.Port.kOnboard; // select roboRIO I2C port
		public static final AMSColorSensor.Address DEFAULT_ADDRESS = AMSColorSensor.Address.DEFAULT;	// I2C address on selected port
		public static enum Color {kBlue, kRed, kWhite};
	}
}
/*

SPECIFICATIONS
Max Operating Voltage: 3.3V
Connectivity: I2C Communication
7-bit I2C Device Address: 0x39
Color Sensor Part: TMD37821
TMD37821 0x39 I²C Vbus = VDD Interface Module-8
Supports Standard (100kHz) or High Speed (400kHz) I2C
Measures: Alpha, Red, Green, Blue, and Proximity
Auto Increment Read: Read All Color and Status Registers with One Call
Built-in IR Proximity Emitter and Detector - 5 -25cm Range
Use Up to Four Color Sensors per REV Robotics Expansion Hub without Additional Hardware Needed
M3 Mounting Hole

Version 1
Rev Robotics
COLOR SENSOR
SKU: REV-31-1154
*/