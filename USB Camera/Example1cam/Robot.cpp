#include "WPILib.h"
#include "PortDefine.h"
#include "IMAQErr.h"
using namespace std;

class QuickVisionRobot : public SampleRobot
{
private:

	USBCamera* mCam;
	Joystick* mXbox;
	Image* mImage;

public:
	QuickVisionRobot();
	void RobotInit();
	void OperatorControl();


};

QuickVisionRobot::QuickVisionRobot()
{
	mCam = new USBCamera("cam0", true);
	mXbox = new Joystick(0);
}

void QuickVisionRobot::RobotInit()
	{
		CameraServer::GetInstance()->SetQuality(50);
		//the camera name (ex "cam0") can be found through the roborio web interface
		CameraServer::GetInstance()->StartAutomaticCapture("cam0");
	}

void QuickVisionRobot::OperatorControl()
	{
	int brightness = 80;
	//mCam->OpenCamera();
	//std::cout << IMAQErrorMessage(-1074360310) << std::endl;
	//mCam->StartCapture();
	while (IsOperatorControl() && IsEnabled())
		{
		mCam->SetBrightness(brightness);
		//mCam->UpdateSettings();
		mCam->GetImage(mImage);
		//CameraServer::GetInstance()->SetImage(mImage);
		std::cout << IMAQErrorMessage(-1074360310) << std::endl;
		std::cout << IMAQErrorMessage(-1074360316) << std::endl;

		printf("Brightness %d\n", mCam->GetBrightness());

			if(mXbox->GetRawButton(XBOX_RIGHT_BUMPER) == true)
			{
				brightness = min(100, brightness + 1);
			}

			if(mXbox->GetRawButton(XBOX_LEFT_BUMPER) == true)
			{
				brightness = max(0, brightness - 1);
			}

			Wait(0.005);				// wait for a motor update time
		}
	}

START_ROBOT_CLASS(QuickVisionRobot);

