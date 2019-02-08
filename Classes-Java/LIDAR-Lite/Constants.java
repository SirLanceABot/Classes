package frc.robot;

import edu.wpi.first.wpilibj.I2C;

public final class Constants {

	public final static class LIDAR {

		public static final I2C.Port PORT = I2C.Port.kMXP /*I2C.Port.kOnboard*/;  // select roboRIO I2C port
		public static final LIDAR_Lite.Address ADDRESS = LIDAR_Lite.Address.DEFAULT;	// I2C address on selected port
		private LIDAR() {}// restrict instantiation with a private constructor - no instantiation is needed for just static constants
	}

	private Constants() {}// restrict instantiation with a private constructor - no instantiation is needed for just static constants
}
