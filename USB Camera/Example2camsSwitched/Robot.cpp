/*
 * Example of running multiple USB cameras off of the RoboRio.
 * FRC Team #116  -  Epsilon Delta
 * Herndon High School
 * Herndon, VA
 *
 * Author:           Mike Anderson
 * Email:            robot_maker12@verizon.net
 * Chief Delphi ID:  taichichuan
 *
 * This is probably worth what you paid for it.  ;-)
 */

#include "WPILib.h"

/**
 * Uses IMAQdx to manually acquire a new image each frame, and annotate the image by drawing
 * a circle on it, and show it on the FRC Dashboard.
 */
class IntermediateVisionRobot: public SampleRobot {
	IMAQdxSession sessionCam0;
	IMAQdxSession sessionCam1;

	Image *frameCam0;
	Image *frameCam1;

	IMAQdxError imaqError;

	// Joystick with which to control the relay.
	Joystick *m_stick;

	// Numbers of the buttons to be used for controlling the Relay.
	const int kCam0Button = 1;
	const int kCam1Button = 2;
	const bool kError = false;
	const bool kOk = true;

public:
	void RobotInit() override {
		m_stick = new Joystick(0); // Use joystick on port 0.
		// create an image
		frameCam0 = imaqCreateImage(IMAQ_IMAGE_RGB, 0);
		frameCam1 = imaqCreateImage(IMAQ_IMAGE_RGB, 0);

	}

	bool StopCamera(int cameraNum) {
		if (cameraNum == 1) {
			// stop image acquisition
			IMAQdxStopAcquisition(sessionCam1);
			//the camera name (ex "cam0") can be found through the roborio web interface
			imaqError = IMAQdxCloseCamera(sessionCam1);
			if (imaqError != IMAQdxErrorSuccess) {
				DriverStation::ReportError(
						"cam1 IMAQdxCloseCamera error: "
								+ std::to_string((long) imaqError) + "\n");
				return (kError);
			}
		} else if (cameraNum == 0) {
			IMAQdxStopAcquisition(sessionCam0);
			imaqError = IMAQdxCloseCamera(sessionCam0);
			if (imaqError != IMAQdxErrorSuccess) {
				DriverStation::ReportError(
						"cam0 IMAQdxCloseCamera error: "
								+ std::to_string((long) imaqError) + "\n");
				return (kError);
			}

		}
		return (kOk);
	}

	bool StartCamera(int cameraNum) {
		if (cameraNum == 1) {
			imaqError = IMAQdxOpenCamera("cam1",
					IMAQdxCameraControlModeController, &sessionCam1);
			if (imaqError != IMAQdxErrorSuccess) {
				DriverStation::ReportError(
						"cam1 IMAQdxOpenCamera error: "
								+ std::to_string((long) imaqError) + "\n");
				return (kError);
			}
			imaqError = IMAQdxConfigureGrab(sessionCam1);
			if (imaqError != IMAQdxErrorSuccess) {
				DriverStation::ReportError(
						"cam0 IMAQdxConfigureGrab error: "
								+ std::to_string((long) imaqError) + "\n");
				return (kError);
			}
			// acquire images
			IMAQdxStartAcquisition(sessionCam1);

		} else if (cameraNum == 0) {
			imaqError = IMAQdxOpenCamera("cam0",
					IMAQdxCameraControlModeController, &sessionCam0);
			if (imaqError != IMAQdxErrorSuccess) {
				DriverStation::ReportError(
						"cam0 IMAQdxOpenCamera error: "
								+ std::to_string((long) imaqError) + "\n");
				return (kError);
			}
			imaqError = IMAQdxConfigureGrab(sessionCam0);
			if (imaqError != IMAQdxErrorSuccess) {
				DriverStation::ReportError(
						"cam0 IMAQdxConfigureGrab error: "
								+ std::to_string((long) imaqError) + "\n");
				return (kError);
			}
			// acquire images
			IMAQdxStartAcquisition(sessionCam0);

		}
		return (kOk);
	}

	void OperatorControl() override {
		bool cam0Btn = false;
		bool cam0Latched = false;
		bool cam1Btn = false;
		bool cam1Latched = false;
		bool cam1FirstTime = true;
		bool cam0FirstTime = true;

		// grab an image, draw the circle, and provide it for the camera server which will
		// in turn send it to the dashboard.
		while (IsOperatorControl() && IsEnabled()) {

			// Retrieve the button values. GetRawButton will return
			//   true if the button is pressed and false if not.
			cam0Btn = m_stick->GetRawButton(kCam0Button);
			cam1Btn = m_stick->GetRawButton(kCam1Button);

			if (cam0Btn || cam0Latched) {
				if (cam1Latched) {
					if (StopCamera(1)) {
						StartCamera(0);
						cam0FirstTime = false;
						cam1Latched = false;
					}
				}

				if (cam0FirstTime) {
					if (StartCamera(0)) {
						cam0FirstTime = false;
					}

				}

				imaqError = IMAQdxGrab(sessionCam0, frameCam0, true, NULL);
				if (imaqError != IMAQdxErrorSuccess) {
					DriverStation::ReportError(
							"cam0 IMAQdxGrab error: "
									+ std::to_string((long) imaqError) + "\n");
				} else {
					CameraServer::GetInstance()->SetImage(frameCam0);
				}
				cam0Latched = true;
			}

			if (cam1Btn || cam1Latched) {
				// stop image acquisition
				if (cam0Latched) {
					if (StopCamera(0)) {
						StartCamera(1);
						cam1FirstTime = false;
						cam0Latched = false;
					}
				}

				if (cam1FirstTime) {
					if (StartCamera(1)) {
						cam1FirstTime = false;
					}

				}

				imaqError = IMAQdxGrab(sessionCam1, frameCam1, true, NULL);
				if (imaqError != IMAQdxErrorSuccess) {
					DriverStation::ReportError(
							"cam0 IMAQdxGrab error: "
									+ std::to_string((long) imaqError) + "\n");
				} else {
					CameraServer::GetInstance()->SetImage(frameCam1);
				}
				cam1Latched = true;

			}

		}
		// stop image acquisition

		if (cam1Btn || cam1Latched) {
			StopCamera(1);
		}
		if (cam0Btn || cam0Latched) {
			StopCamera(0);
		}
	}
};

START_ROBOT_CLASS(IntermediateVisionRobot);

