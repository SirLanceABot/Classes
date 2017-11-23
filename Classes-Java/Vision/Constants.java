package org.usfirst.frc.team4237.robot;

public final class Constants {

	public final static class LOCKS {

		public static final Object cameraThread = new Object(); // need an object to lock camera_process - this will do

		private LOCKS() {}// restrict instantiation with a private constructor - no instantiation is needed for just static constants
	}

	private Constants() {}// restrict instantiation with a private constructor - no instantiation is needed for just static constants
}
