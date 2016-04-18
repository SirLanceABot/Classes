#include "ITG-3200.h"
#include "WPILib.h"
#include "Constants.h"
#include "XboxJoystick.h"

ITG3200::ITG3200(I2C::Port port, uint8_t deviceAddress, int sampleRate, double initialAngle[], float sensitivity, double integrationRate) :
mGyro (new I2C(port, deviceAddress)),
m_backgroundLoop (std::make_unique<Notifier>(&ITG3200::Get, this)),
mTimer (new Timer()),
mSensitivity (sensitivity),
mIntegrationRate(integrationRate)
{
	printf("File %18s Date %s Time %s Object %p\n",__FILE__,__DATE__, __TIME__, this);

	mAngle[0] = 0.0;
	mAngle[1] = 0.0;
	mAngle[2] = 0.0;

	mTimer->Start();

	// printf("Status=:%#.2x\n", mBuffer[0]);

	Wait(.1);

	mGyro->Write(ITG3200_PWR_MGM, ITG3200_RESET);

	Wait(.1);

	mGyro->Read(ITG3200_ID_REGISTER, 1, mBuffer);

	mWorking = (int8_t)(mBuffer[0] & ITG3200_ID_BITS) >> 1 == ITG3200_ID;

	if (IsWorking())
	{
		printf("[ITG-3200] gyro working on port %x, address %x, device ID register %x\n",
				port, deviceAddress, mBuffer[0]);

		SetSampleRate(sampleRate);

		Wait(1.0);

		printf("[ITG-3200] starting calibration - do not vibrate gyro until completed\n");

		Calibrate();

		SetAngle(initialAngle);
	}
	else
	{
		printf("[ITG-3200] gyro NOT working on port %x, address %x, device ID register %x\n", port, deviceAddress, mBuffer[0]);
	}
}

ITG3200::~ITG3200()
{
	delete mTimer;
	m_backgroundLoop->Stop(); // Notifier Stop doesn't return here until last loop has completed
	delete mGyro;
}

bool ITG3200::IsWorking()
{
	return mWorking;
}

void ITG3200::Calibrate()
{
	int reads = 600;
	double delay = 0.004; // 4 milliseconds
	int skip = 5; // initial samples to skip
	double temp[3] = {0.0, 0.0, 0.0};

	m_backgroundLoop->Stop(); // Notifier Stop doesn't return here until last loop has completed, okay to Stop if not started

	for(int i = 0; i < reads; i++)
	{
		GetRaw();
		if (i >= skip)
		{
			for (int j = 0; j < 3; j++)
			{
				temp[j] += mRotation[j];
			}
		}
		Wait(delay);
	}

	printf("[ITG-3200] completed calibrated offset of X Y Z: ");

	for(int i = 0; i < 3; i++)
	{
		mOffset[i] = -(float)temp[i] / (float)(reads - skip);
		printf(" %f, ", mOffset[i]);
	}
	printf("\n");

	m_backgroundLoop->StartPeriodic(mIntegrationRate); // background function starts running
}

void ITG3200::Get()
{
	double current_time = mTimer->Get();

	if (current_time < mPrevTime)
		mPrevTime = current_time - mIntegrationRate; // some clocks rollover often; use expected time step

	GetRaw();

	std::lock_guard<priority_mutex> ProtectMe(m_mutex); // protect the shared memory from others' access while it's being updated here

	for(int i = 0; i < 3; i++)
	{
		mAngleRate[i] = (mRotation[i] + mOffset[i])/mSensitivity; // convert raw sensor to angular rate
		mAngle[i] += mAngleRate[i] * (current_time - mPrevTime); // integrate angular rate to yield angle
	}

	mPrevTime = current_time;
}

void ITG3200::GetAngle(double angle[])
{
	std::lock_guard<priority_mutex> ProtectMe(m_mutex); // protect the shared memory from others' access while it's being read here
	angle[0] = mAngle[0];
	angle[1] = mAngle[1];
	angle[2] = mAngle[2];
}

void ITG3200::GetAngleRate(double angleRate[])
{
	std::lock_guard<priority_mutex> ProtectMe(m_mutex); // protect the shared memory from others' access while it's being read here
	angleRate[0] = mAngleRate[0];
	angleRate[1] = mAngleRate[1];
	angleRate[2] = mAngleRate[2];
}

float ITG3200::GetTemperature()
{
	std::lock_guard<priority_mutex> ProtectMe(m_mutex); // protect the shared memory from others' access while it's being read here
	return 35.0 + (mTemperature + 13200.0) / 280.0;
}

void ITG3200::SetAngle(double angle[])
{
	std::lock_guard<priority_mutex> ProtectMe(m_mutex); // protect the shared memory from others' access while it's being updated here
	mAngle[0] = angle[0];
	mAngle[1] = angle[1];
	mAngle[2] = angle[2];
	mPrevTime = mTimer->Get();
}

void ITG3200::SetSampleRate(uint8_t sampleRate) // sampleRate is the digital Low Pass Filter Configuration
{
	uint8_t dLPFFS;	//This is the digital low pass filter and full scale

	if (sampleRate < 0 || sampleRate > 7)
		sampleRate = 0;

	dLPFFS = ITG3200_FS_3 | sampleRate;

	mGyro->Write(ITG3200_DLPF_FS, dLPFFS);

	printf("[ITG-3200] DLPF, Full Scale, %#.2x\n", dLPFFS);

	Wait(0.06);
}

void ITG3200::GetOffset(double offset[3])
{
	offset[0] = mOffset[0];
	offset[1] = mOffset[1];
	offset[2] = mOffset[2];
}

double ITG3200::GetZ()
{
	double angle[3];
	GetAngle(angle);
	return angle[2];
}

double ITG3200::GetX()
{
	double angle[3];
	GetAngle(angle);
	return angle[0];
}

double ITG3200::GetY()
{
	double angle[3];
	GetAngle(angle);
	return angle[1];
}

void ITG3200::Test(std::shared_ptr<XboxJoystick> xbox)
{
	double angle[3];
	double angleRate[3];
	GetAngle(angle);
	GetAngleRate(angleRate);
	float leftY;
		bool printMenu = true;

		while(xbox->GetX() == false)
		{
			if(printMenu == true)
			{
				printf("Press X to Exit\n");
				printf("Press A and Left Y to display gyro\n");
				printMenu = false;
			}

			while(xbox->GetA())
			{
				leftY = xbox->GetLeftYAxis();
				printMenu = true;
				if(leftY != 0 && xbox->GetA())
				{
					GetAngle(angle);
					GetAngleRate(angleRate);
					printf("[ITG3200] temperature: %7.2f, X: %9.2f, Y: %9.2f, Z: %9.2f, Xrate: %9.2f, Yrate: %9.2f, Zrate: %9.2f\n",
							GetTemperature(),
							angle[0], angle[1], angle[2],
							angleRate[0], angleRate[1], angleRate[2]);
				}
			}
		}

}

void ITG3200::GetRaw()
{
	mGyro->Read(ITG3200_TEMP_H, 8, mBuffer);
	mRotation[0] = (float)(int16_t)(((mBuffer[2] << 8 ) | mBuffer[3]));	// x
	mRotation[1] = (float)(int16_t)(((mBuffer[4] << 8 ) | mBuffer[5]));	// y
	mRotation[2] = (float)(int16_t)(((mBuffer[6] << 8 ) | mBuffer[7]));	// z

	std::lock_guard<priority_mutex> ProtectMe(m_mutex); // protect the shared memory from others' access while it's being updated here
	mTemperature = (float)(int16_t)(((mBuffer[0] << 8 ) | mBuffer[1]));  // temperature

	//			printf("Temperature %#.2x %#.2x %f\n",
	//					mBuffer[0], mBuffer[1], mTemperature);
	//			printf("X %#.2x %#.2x %f Y %#.2x %#.2x %f Z %#.2x %#.2x %f\n",
	//					mBuffer[2], mBuffer[3], mRotation[0], mBuffer[4], mBuffer[5], mRotation[1], mBuffer[6], mBuffer[7], mRotation[2]);
}
