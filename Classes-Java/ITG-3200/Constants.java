package org.usfirst.frc.team4237.robot;

import edu.wpi.first.wpilibj.I2C;

public final class Constants {

	public final static class GYRO {

		public static final I2C.Port SIX_DOF_PORT = I2C.Port.kMXP /*I2C.Port.kOnboard*/;
		public static final ITG3200.Address ADDRESS= ITG3200.Address.DEFAULT; // I2C address of the gyro
		public static final ITG3200.DLPF_Cfg SAMPLE_RATE_FILTER = ITG3200.DLPF_Cfg.k1000Hz_188Hz; // gyro internal sampling and filtering rate
		public static final float GYRO_SENSITIVITY = (float) (0.995 * ITG3200.SENSITIVITY_SCALE_FACTOR);
		// todo: gyro sensitivity needs to be checked/calibrated; typically multiply default by a number near 1.0 to adjust accurately
		// Sensitivity is a divisor so larger values yield less turning angle for a rotation
		// if the robot turns 90 degrees but the gyro indicates 89 degrees then decrease the sensitivity by 89/90
		public static final double[] INITIAL_ANGLE = {0.0,0.0,0.0}; // angle of the gyro (or robot) when it is initialized

		private GYRO() {}// restrict instantiation with a private constructor - no instantiation is needed for just static constants
	}

	private Constants() {}// restrict instantiation with a private constructor - no instantiation is needed for just static constants
}
