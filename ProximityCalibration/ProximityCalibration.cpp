/*
 * Calibrate Proximity Sensors - IR or US (converts voltage to engineering units)
 *
 * sample proximity sensor voltages at various distances
 * get many voltage samples at each distance and average them
 * compute curve fits of the measured data
 * graph results
 *
 * Set defines for the Analog Input Port of the sensor,
 * optional extra print of sample statistics (standard deviation, kertosis, etc), and
 * number of samples to be curve fit and rebuild and deploy
 *
 * Enable TeleOp mode on the DriverStation
 *
 * On the SmartDashboard type the sensor ID and hit tab (NOT Enter) to the
 * "Type New Sample Inches -  Hit tab"
 *
 * Type the distance and remove the -999 from the input field
 *
 * Hit Tab
 *
 * The sample will be processed in a second and a new -999 will appear.
 * Tab to that input field and repeat data entry for all samples
 *
 * Results appear on the "NetConsole" (riolog, Console, NetConsole replacements)
 *
 */

#define AnalogInputPort 2
#undef PrintSampleStats
#define MAX_SAMPLES 8

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string>
#include "WPILib.h"
#include "stats_running.h"

class Robot: public SampleRobot
{
AnalogInput rangefinder;

public:
	Robot() : rangefinder(AnalogInputPort){} // argument is AnalogInput port of the proximity sensor

void OperatorControl()
	{
	// user interface variables
	double InputDistance, lastDistance, lastVoltage;
	std::string SensorID = "Enter Sensor ID";
    std::string Title;

	// process samples
    double voltageSample;
    RunningStats voltageFuzzy;
    char header[200];
    double voltageSmoothed[MAX_SAMPLES];
    double ActualDistance[MAX_SAMPLES];

	// curve fit objects
    RunningLeastSquares Sensor;
    RunningRegression SensorLinear;

	// curve fit results
    double PowerDistance[MAX_SAMPLES];
    double LogarithmicDistance[MAX_SAMPLES];
    double ExponentialDistance[MAX_SAMPLES];
    double LinearDistance[MAX_SAMPLES];

	// graphical output
    char line[101][101];

//    Timer timer;

	/***************************************************
	Get distance/voltage samples
	***************************************************/

	printf("\n\nS T A R T I N G   S E N S O R   C A L I B R A T I O N\n\n"
			   "                  Analog Input Port %d\n\n", AnalogInputPort);

	sprintf(header, "Enter %d Samples", MAX_SAMPLES);

	SmartDashboard::PutString("Number of Samples", header);
	SmartDashboard::PutString("Sensor ID", SensorID);
	lastDistance = lastVoltage = -999.;

    for (int measurementCount = 0; measurementCount < MAX_SAMPLES; measurementCount++) // distance/voltage measurements
	{
	// display previous sample
	SmartDashboard::PutNumber("Last Sample Voltage", lastVoltage);
	SmartDashboard::PutNumber("Last Sample Inches", lastDistance);
	SmartDashboard::PutNumber("Analog Voltage", rangefinder.GetVoltage());

	// get the new sample distance
	SmartDashboard::PutNumber("Entering Sample Number", measurementCount+1);
	InputDistance = -999.;
	SmartDashboard::PutNumber("Type New Sample Inches -  Hit tab", InputDistance);
	//timer.Start();
	while (InputDistance <= 0.0) // wait for a distance to be entered
	{
		if(IsDisabled() || !IsOperatorControl()) // allow user to break out before any sample
		{
			printf("\n\nB R E A K I N G   S E N S O R   C A L I B R A T I O N   -   N O T   C O M P L E T E D\n\n");

			return;
		}

		if (SensorID == "Enter Sensor ID")
		{
			SensorID = frc::SmartDashboard::GetString("Sensor ID", "Enter Sensor ID");
			if (SensorID != "Enter Sensor ID") std::cout << "                 Sensor ID """ << SensorID << """\n\n";
		}

		//printf("%f, %d, %f\n", timer.Get(), __LINE__, InputDistance);
		SmartDashboard::PutNumber("Analog Voltage", rangefinder.GetVoltage());
		InputDistance = frc::SmartDashboard::GetNumber("Type New Sample Inches -  Hit tab", -999.L);
		Wait(.05);
	}

	ActualDistance[measurementCount] = InputDistance;  // new distance has been entered

	// get the corresponding voltage - average many samples at this distance to quiet the noise
	voltageFuzzy.Clear(); // reset voltage average calculator

	for (int i=0; i <50; i++) // get 50 voltage samples at this distance and average them
	    {
	    voltageSample = rangefinder.GetVoltage();  // get a sample [maybe use GetAverageVoltage() when implementing drive]
	    voltageFuzzy.Push(voltageSample); // add sample to average
	    Wait(.02);  // Wait a bit to take another sample voltage at this distance
	    }

	voltageSmoothed[measurementCount] = voltageFuzzy.Mean();  // get the average voltage at this distance

	printf("\nSample %d   Smoothed Voltage: %g  |  Measured Distance: %g\n",
			measurementCount+1, voltageSmoothed[measurementCount], ActualDistance[measurementCount]);

	// print statistics on the voltage samples for this distance
#if defined (PrintSampleStats)
	voltageFuzzy.Print(stdout, "Voltage Statistics");
#endif
	Sensor.Push(voltageSmoothed[measurementCount], ActualDistance[measurementCount]);  // save the voltage and distance for the non-linear curve fits

	SensorLinear.Push(voltageSmoothed[measurementCount], ActualDistance[measurementCount]);  // save the voltage and distance for the linear curve fit

	lastVoltage = voltageSmoothed[measurementCount]; // save this sample to display
	lastDistance = ActualDistance[measurementCount];
	}
	/***************************************************
	Completed all distance/voltage samples
	***************************************************/

	/***************************************************
	Compute least squares fits
	***************************************************/

    Title = "non-linear least squares fits for sensor " + SensorID;
    Sensor.Print(stdout, Title.c_str());

    Title = "linear (first-order) least squares fit for sensor " + SensorID;
    SensorLinear.Print(stdout, Title.c_str());

    printf("\nsample#, average voltage, measured distance, power distance, logarithmic distance, exponential distance, linear distance\n");

    // make a table with the voltages of the distance samples and the measured and curve fit distances
    for (int measurementCount = 0; measurementCount < MAX_SAMPLES; measurementCount++)
	{
	PowerDistance[measurementCount] = Sensor.Power_A()*pow(voltageSmoothed[measurementCount], Sensor.Power_B());

	LogarithmicDistance[measurementCount] = Sensor.Logarithmic_A() + Sensor.Logarithmic_B()*log(voltageSmoothed[measurementCount]);

	ExponentialDistance[measurementCount] = Sensor.Exponential_A()*exp(Sensor.Exponential_B()*voltageSmoothed[measurementCount]);

	LinearDistance[measurementCount] = SensorLinear.Intercept() + SensorLinear.Slope()*voltageSmoothed[measurementCount];

	printf("%d,\t\t\t%.4g,\t\t\t\t%.4g,\t\t\t\t\t%.4g,\t\t\t%.4g,\t\t\t\t%.4g,\t\t\t\t%.4g\n",
		measurementCount+1, voltageSmoothed[measurementCount], ActualDistance[measurementCount], PowerDistance[measurementCount],
		LogarithmicDistance[measurementCount], ExponentialDistance[measurementCount], LinearDistance[measurementCount] );
	}

    // initialize the graph
    for (int axisDistance = 0; axisDistance <= 100; axisDistance++)
	for (int axisVoltage = 0; axisVoltage <= 100; axisVoltage++)
	    line[axisDistance][axisVoltage] = axisVoltage%2 == 0 ? '.': ' ';

    // locate the points on the graph
	for (int measurementCount = 0; measurementCount < MAX_SAMPLES; measurementCount++)
	    {
	    int axisVoltage = std::min(std::max((int)(voltageSmoothed[measurementCount]*20. + .5), 0), 100);

		#define AxisDistanceScale(d) std::min(std::max((int)(d*2. + .5), 0), 100)

		line[AxisDistanceScale(LinearDistance     [measurementCount])][axisVoltage] = '1';

		line[AxisDistanceScale(LogarithmicDistance[measurementCount])][axisVoltage] = 'L';

		line[AxisDistanceScale(ExponentialDistance[measurementCount])][axisVoltage] = 'E';

		line[AxisDistanceScale(PowerDistance      [measurementCount])][axisVoltage] = 'P';

		line[AxisDistanceScale(ActualDistance     [measurementCount])][axisVoltage] = 'M';
	    }

	/***************************************************
	print graph
	***************************************************/
    for (int axisDistance = 100; axisDistance >= 0; axisDistance--)
	{
	printf("%101.101s\n", line[axisDistance]);
	Wait(.02);
	}

	printf("\n\nE N D I N G   S E N S O R   C A L I B R A T I O N   -   C O M P L E T E D\n\n");

	fflush(stdout);

	/**************************************************
	end OperatorControl method
	**************************************************/
	}

/**************************************************
end robot class
**************************************************/
};

/**************************************************
start main
**************************************************/

START_ROBOT_CLASS(Robot);
