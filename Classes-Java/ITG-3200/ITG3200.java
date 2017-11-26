package org.usfirst.frc.team4237.robot;

import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.I2C;

public class ITG3200 {

	/**
	 * Last parameter for integrationRate not specified.
	 * 
	 * <p>Using default integrationRate period 0.015 seconds
	 */
	public ITG3200(I2C.Port port, Address deviceAddress, ITG3200.DLPF_Cfg sampleRate, double[] initialAngle, float sensitivity)
	{
		this(port, deviceAddress, sampleRate, initialAngle, sensitivity, 13, 0.015 /* default sampleRateDivider (- 1 of actual) and integrationRate period in seconds*/);
		//                                                 Consider sampleRateDivider to be 2 less than the integration period in milliseconds
	}

	public ITG3200(I2C.Port port, Address deviceAddress, ITG3200.DLPF_Cfg sampleRate, double[] initialAngle, float sensitivity, int sampleRateDivider, double integrationRate)
	{
		byte[] deviceID = new byte[1];

		System.out.println("[ITG-3200] starting gyro class");
		this.mGyro = new I2C(port, deviceAddress.value);
		mTimer = new Timer();
		this.mSensitivity = sensitivity;
		this.mWorking = true;
		this.mIntegrationRate = integrationRate;
		this.mPrevTime = 0;

		mAngle[0] = 0.0; // overridden by set initial angle but initialize in case used as a side effect of calibrate (calling Get instead of GetRaw)
		mAngle[1] = 0.0;
		mAngle[2] = 0.0;

		edu.wpi.first.wpilibj.Timer.delay(1.0);

		// First, reset gyro - sets all default values in gyro

		if (mGyro.write(Register.PWR_MGM.value, PWR_MGM.H_RESET.value))
		{
			mWorking = false;
			System.out.println("[ITG-3200] reset failed");
		}

		if (IsWorking())
		{
			edu.wpi.first.wpilibj.Timer.delay(1.0);

			System.out.println("[ITG-3200] turning off x and y gyros.  clocking on z gyro.");
			if (mGyro.write(Register.PWR_MGM.value, PWR_MGM.STBY_XG.value + PWR_MGM.STBY_YG.value + PWR_MGM.CLK_SEL_ZGyro.value))
			{
				mWorking = false;
				System.out.println("[ITG-3200] power setting failed");
			}
		}


		if (IsWorking())
		{
			edu.wpi.first.wpilibj.Timer.delay(1.0);
			if (mGyro.read(Register.WHO_AM_I.value, deviceID.length, deviceID))
			{
				mWorking = false;
				System.out.println("[ITG-3200] read ID failed");
			}
			else
			{ 
				mWorking = ((deviceID[0] & WhoAmIMask_ID) >> 1) == Who_Am_I_ID; // is it the right ID or not
			}
		}

		if (IsWorking())
		{
			System.out.printf("[ITG-3200] gyro working on port %s, address %s %#x, device ID register %#x\n", port, deviceAddress, deviceAddress.value, deviceID[0]);

			SetRawDataReady();
			edu.wpi.first.wpilibj.Timer.delay(0.1);

			SetSampleRate(sampleRate.value);
			edu.wpi.first.wpilibj.Timer.delay(0.1);

			SetSampleRateDivider(sampleRateDivider);
			edu.wpi.first.wpilibj.Timer.delay(0.1);

			Calibrate();

			SetAngle(initialAngle); // must be after Calibrate since angles changed if using Get in it instead of GetRaw
		}
		else
		{
			System.out.printf("[ITG-3200] gyro NOT working on port %s, address %s %#x, device ID register %#x\n", port, deviceAddress, deviceAddress.value, deviceID[0]);
		}
	}

	private void Calibrate()
	{
		System.out.print("[ITG-3200] starting calibration - do not vibrate gyro until completed\n");

		Stop(); // stop the background thread

		int reads = 600;  // number of samples for calibration
		double delay = 0.005; // wait 5 milliseconds between samples
		int skip = 5; // initial samples to skip
		double[] temp = {0.0, 0.0, 0.0};

		edu.wpi.first.wpilibj.Timer.delay(1.);  // give people very little time to settle

		for (int i = 0; i < reads; i++)
		{
			Get(); // only need GetRaw but handy to use the check data ready logic in Get so use that
			if (i >= skip)
			{
				for (int j = 0; j < 3; j++)
				{
					temp[j] += mRotation[j];
				}
			}
			edu.wpi.first.wpilibj.Timer.delay(delay);
		}

		System.out.print("[ITG-3200] completed calibrated offset of X Y Z: ");

		for (int i = 0; i < 3; i++)
		{
			mOffset[i] = -(float)temp[i] / (float)(reads - skip);
			System.out.printf(" %f, ", mOffset[i]);
		}
		System.out.println();

		StartPeriodic(mIntegrationRate); // start the background thread
	}

	private void Stop() {
		if (m_backgroundLoop != null) m_backgroundLoop.cancel();  // stop scheduler if it is running
	}

	private void StartPeriodic(double rate)
	{
		long loopTime = (long)(rate*1000. + .5);
		m_backgroundLoop = new UpdateGyro();
		mPrevTime = System.currentTimeMillis();
		mTimer.schedule(m_backgroundLoop, // TimerTask to schedule to run
				0,        // initial delay is 0 - run immediately
				loopTime);  // subsequent period milliseconds
		System.out.println("[ITG-3200] Background task running every " + loopTime + " milliseconds");
	}

	public boolean IsWorking()
	{
		return mWorking;
	}

	private class UpdateGyro extends TimerTask
	{
		public void run() {
			Get(); // keep run() minimal - Get is where all the action is
		}
	}

	private void Get()
	{
		long current_time;

		// check for new value available on the gyro; wait awhile if no new data
		for(int i=1;i <= 20; i++)
		{
			if(!GetRawDataReady())
			{
				edu.wpi.first.wpilibj.Timer.delay(0.001);
			}
			else
			{
				if(i > 1 && (m_backgroundLoop != null)) System.out.println("[ITG-3200] data was not ready " + i); // if short wait or in Calibrate, skip messages
				break;
			}
		}

		current_time =  System.currentTimeMillis(); // System.nanoTime() seems unnecessary

		GetRaw();

		synchronized(this)
		{
			for (int i = 0; i < 3; i++)
			{
				mAngleRate[i] = (mRotation[i] + mOffset[i]) / mSensitivity; // convert raw sensor to angular rate
				mAngle[i] += mAngleRate[i] * (float)(current_time - mPrevTime) * 0.001F; // time integrate angular rate to yield angle
			}
		}

		mPrevTime = current_time;
	}

	private boolean GetRawDataReady()
	{
		byte[] deviceStatus = new byte[1];

		if (mGyro.read(Register.INT_STATUS.value, deviceStatus.length, deviceStatus))
		{
			mWorking = false;
			System.out.println("[ITG-3200] read interrupt status failed");
			return false;
		}
		else
		{ 
			return (deviceStatus[0] & 0xFF) == RAW_DATA_RDY;
		}
	}

	public void GetAngle(double[] angle)
	{
		synchronized(this)
		{
			angle[0] = mAngle[0];
			angle[1] = mAngle[1];
			angle[2] = mAngle[2];
		}
	}

	public void GetAngleRate(double[] angleRate)
	{
		synchronized(this)
		{
			angleRate[0] = mAngleRate[0];
			angleRate[1] = mAngleRate[1];
			angleRate[2] = mAngleRate[2];
		}
	}

	public float GetTemperature()
	{
		synchronized(this)
		{
			return (float)(35.0 + (mTemperature + 13200.0) / 280.0);
		}
	}

	public void SetAngle(double[] angle)
	{
		synchronized(this)
		{
			mAngle[0] = angle[0];
			mAngle[1] = angle[1];
			mAngle[2] = angle[2];
			mPrevTime = System.currentTimeMillis();
		}
	}

	private void SetRawDataReady()
	{
		if (mGyro.write(Register.INT_CFG.value, RAW_RDY_EN))
		{
			mWorking = false;
			System.out.print("[ITG-3200] write RawDataReady interrupt configuration failed\n");
		}
		else
		{
			System.out.printf("[ITG-3200] RawDataReady interrupt set\n");
		}
	}

	public void SetSampleRate(int aDLPF_Cfg)
	{
		int DLPF_FS; //This is the digital low pass filter and full scale - 2 parts of 1 register

		if (aDLPF_Cfg < DLPF_Cfg.k8000Hz_256Hz.value || aDLPF_Cfg > DLPF_Cfg.k1000Hz_5Hz.value) // Validate argument is within range
		{
			aDLPF_Cfg = DLPF_Cfg.k8000Hz_256Hz.value; // if bad argument then set to the default
		}

		DLPF_FS = FS_SEL | aDLPF_Cfg; // combine both parts of DLPS_FS

		if (mGyro.write(Register.DLPF_FS.value, DLPF_FS))
		{
			mWorking = false;
			System.out.print("[ITG-3200] write configuration failed\n");
		}
		else
		{
			System.out.printf("[ITG-3200] DLPF, Full Scale, %#x\n", DLPF_FS);
		}
	}

	private void SetSampleRateDivider(int sampleRateDivider) // private since this rate should be tied to the scheduled sample rate but no method provided to change that
	{
		if (sampleRateDivider < 0 || sampleRateDivider > 255)
		{
			sampleRateDivider = 0;
		}

		if (mGyro.write(Register.SMPLRT_DIV.value , sampleRateDivider))
		{
			mWorking = false;
			System.out.print("[ITG-3200] write sample rate divider failed\n");
		}
		else
		{
			System.out.printf("[ITG-3200] sample rate divider + 1 = %d\n", sampleRateDivider + 1);
		}
	}

	//	public void GetOffset(double[] offset)
	//	{
	//		offset[0] = mOffset[0];
	//		offset[1] = mOffset[1];
	//		offset[2] = mOffset[2];
	//	}

	public double GetX()
	{
		double[] angle = new double[3];
		GetAngle(angle);
		return angle[0];
	}

	public double GetY()
	{
		double[] angle = new double[3];
		GetAngle(angle);
		return angle[1];
	}

	public double GetZ()
	{
		double[] angle = new double[3];
		GetAngle(angle);
		return angle[2];
	}

	public String toString()
	{
		double[] angle = new double[3];
		double[] angleRate = new double[3];

		GetAngle(angle);
		GetAngleRate(angleRate);

		if (IsWorking())
			return String.format("[ITG-3200] (temperature= %7.2f) (X=%9.2f, Y=%9.2f, Z=%9.2f) (Xrate=%9.2f, Yrate=%9.2f, Zrate=%9.2f)",
					GetTemperature(),
					angle[0], angle[1], angle[2],
					angleRate[0], angleRate[1], angleRate[2]);

		else
			return String.format("[ITG-3200] gyro NOT working");
	}

	private void GetRaw()
	{
		byte[] rawData = new byte[DATA_REG_SIZE];

		// the first register to read is specified;
		// the rest are retrieved in order by the I2C driver until the count number of bytes are read	
		if (mGyro.read(FIRST_DATA_REGISTER_TO_READ, rawData.length, rawData))
		{
			System.out.print("[ITG-3200] GetRaw Read failed\n");
			for (int i = 0; i < rawData.length; i++)
			{
				rawData[i] = 0;
			}
		}
		// how to turn 2 Java bytes into 1 Java int (gyro 2 bytes are slllllll rrrrrrrr)
		// left byte is converted to implied int conserving the sign and shifted to make room for the right byte.
		// slllllll => ssssssss ssssssss ssssssss slllllll => ssssssss ssssssss slllllll 00000000
		// right byte is converted to implied int conserving the sign that we don't want so wipe it out with AND 000000FF. (the sign comes from the first r as Java erroneously thinks)
		// rrrrrrrr => ssssssss ssssssss ssssssss rrrrrrrr => 00000000 00000000 00000000 rrrrrrrr 
		// then OR (add) the 2 implied int pieces to make an implied int number. ssssssss ssssssss slllllll rrrrrrrr
		mRotation[0] = (float)((rawData[2] << 8) | (rawData[3] & 0xFF)); // x
		mRotation[1] = (float)((rawData[4] << 8) | (rawData[5] & 0xFF)); // y
		mRotation[2] = (float)((rawData[6] << 8) | (rawData[7] & 0xFF)); // z
		synchronized(this)
		{
			mTemperature = (float)(((rawData[0] << 8) | (rawData[1] & 0xFF))); // temperature
		}
		//		System.out.printf("[ITG-3200] Temperature %x %x %f\n",
		//				rawData[0], rawData[1], mTemperature);
		//		System.out.printf("[ITG-3200] X %x %x %f Y %x %x %f Z %x %x %f\n",
		//				rawData[2], rawData[3], mRotation[0], rawData[4], rawData[5], mRotation[1], rawData[6], rawData[7], mRotation[2]);
	}

	/*
		Methods not implemented since they don't make much sense to do so for a gyro sensor:
			public int hashCode()
			public boolean equals(Object obj)
			copyFrom
			clone
	 */

	// I2C addresses
	public static enum Address { // I2C addresses
		DEFAULT(0x68), // default I2C bus address for the LIDAR Lite v2
		ALTERNATE(0x69);
		public final int value;
		private Address(int value){
			this.value = value; // need this if "casting" Address to int for "external" usage, array size, etc
		}
	}

	// default sensitivity
	public static final float SENSITIVITY_SCALE_FACTOR = 14.375F;

	// Register Map
	private static enum Register {
		WHO_AM_I   (0x00), // Who Am I
		SMPLRT_DIV (0x15), // Sample Rate Divider
		DLPF_FS    (0x16), // Digital Low Pass Filter and Full Scale configuration
		INT_CFG    (0x17), // Interrupt Configuration
		INT_STATUS (0x1A), // Interrupt Status
		TEMP_OUT_H (0x1B), // Sensor data (read only)
		TEMP_OUT_L (0x1C), // Sensor data (read only)
		GYRO_XOUT_H(0x1D), // Sensor data (read only)
		GYRO_XOUT_L(0x1E), // Sensor data (read only)
		GYRO_YOUT_H(0x1F), // Sensor data (read only)
		GYRO_YOUT_L(0x20), // Sensor data (read only)
		GYRO_ZOUT_H(0x21), // Sensor data (read only)
		GYRO_ZOUT_L(0x22), // Sensor data (read only)
		PWR_MGM    (0x3E); // Power Management
		public final int value;
		private Register(int value){
			this.value = value; // need this if "casting" Register to int for "external" usage and array size
		}
	}

	// Who Am I ID
	private static final int WhoAmIMask_ID = 0x00_00_00_7e; // mask
	private static final int Who_Am_I_ID = 0x34; // expected ID of gyro

	// FS=11 DLPF=000 => 11000 => 0x18 => 0b11'000
	private static final int FS_SEL = 0x18; // 3 = +- 2000 deg/sec only value allowed by ITG-3200

	// sample rate and filtering constants
	public static enum DLPF_Cfg {
		k8000Hz_256Hz(0), // 256Hz low pass filter bandwidth,    8,000Hz Internal Sample Rate
		k1000Hz_188Hz(1), // 188Hz low pass filter bandwidth,    1,000Hz Internal Sample Rate
		k1000Hz_98Hz(2),  //  98Hz low pass filter bandwidth,    1,000Hz Internal Sample Rate
		k1000Hz_42Hz(3),  //  42Hz low pass filter bandwidth,    1,000Hz Internal Sample Rate
		k1000Hz_20Hz(4),  //  20Hz low pass filter bandwidth,    1,000Hz Internal Sample Rate
		k1000Hz_10Hz(5),  //  10Hz low pass filter bandwidth,    1,000Hz Internal Sample Rate
		k1000Hz_5Hz(6);   //   5Hz low pass filter bandwidth,    1,000Hz Internal Sample Rate
		public final int value;
		private DLPF_Cfg(int value){
			this.value = value; // need this if "casting" to int for "external" usage and array size
		}
	}

	// Interrupt Configuration
	private static final int RAW_RDY_EN = 0x01;

	// Interrupt Status
	private static final int RAW_DATA_RDY = 0x01;

	// Sensor Data
	// Warning - "Fancy" way to robustly determine what data to read but that's all ruined by building in the
	// subscripts throughout the program to extract the data.  Changes here still have to be made in a few other places (got tired)
	private static final int FIRST_DATA_REGISTER_TO_READ = Register.TEMP_OUT_H.value;
	private static final int LAST_DATA_REGISTER_TO_READ = Register.GYRO_ZOUT_L.value;
	private static final int DATA_REG_SIZE = LAST_DATA_REGISTER_TO_READ - FIRST_DATA_REGISTER_TO_READ + 1;

	// Power Management
	private static enum PWR_MGM { // additive values to set multiple options
		H_RESET(0x80), // additive values to set multiple options
		SLEEP(0x40),   // additive values to set multiple options
		STBY_XG(0x20), // additive values to set multiple options
		STBY_YG(0x10), // additive values to set multiple options
		STBY_ZG(0x08), // additive values to set multiple options
		CLK_SEL_Internal(0),       // select one clock select - prefer one of the gyros
		CLK_SEL_XGyro(1),          // select one clock select - prefer one of the gyros
		CLK_SEL_YGyro(2),          // select one clock select - prefer one of the gyros
		CLK_SEL_ZGyro(3),          // select one clock select - prefer one of the gyros
		CLK_SEL_External_32kHz(4), // select one clock select - prefer one of the gyros
		CLK_SEL_External_19kHz(5), // select one clock select - prefer one of the gyros
		WAKEUP(0x00); // clears everything - add other options
		public final int value;
		private PWR_MGM(int value){
			this.value = value; // need this if "casting" Address to int for "external" usage, array size, etc
		}
	}

	// Class Global variables
	private I2C mGyro;
	private Timer mTimer;
	private TimerTask m_backgroundLoop;
	private float mSensitivity;
	private double[] mAngle = new double[3];
	private float[] mOffset = new float[3];
	private float[] mAngleRate = new float[3];
	private float mTemperature;
	private float[] mRotation = new float[3];
	private boolean mWorking;
	private double mIntegrationRate;
	private long mPrevTime;
}
