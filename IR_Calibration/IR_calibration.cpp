// sample IR proximity sensor voltages at various distances
// get many voltage samples at each distance and average them.
// compute curve fits of the measured data
// graph results

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "WPILib.h"
#include "stats_running.h"

class Robot: public SampleRobot
{
AnalogInput rangefinder;

public:
	Robot() : rangefinder(2){} // argument is AnalogInput port of the IR proximity sensor

void OperatorControl()
	{
	#define MAX_SAMPLES 8

	// user interface variables
	double InputDistance, lastDistance, lastVoltage;
	
	// process samples
    double voltageSample;
    RunningStats voltageFuzzy;
    char header[200];
    double voltageSmoothed[MAX_SAMPLES];
    double ActualDistance[MAX_SAMPLES];
	
	// curve fit objects
    RunningLeastSquares IR;
    RunningRegression IRLinear;
	
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
	
	printf("\n\nS T A R T I N G   I R   C A L I B R A T I O N\n\n");
	
	sprintf(header, "Enter %d Samples", MAX_SAMPLES);
	
	SmartDashboard::PutString("Number of Samples", header);

	lastDistance = lastVoltage = -999.;

    for (int measurementCount = 0; measurementCount < MAX_SAMPLES; measurementCount++) // distance/voltage measurements
	{
	// display previous sample
	SmartDashboard::PutNumber("Last Sample Voltage", lastVoltage);
	SmartDashboard::PutNumber("Last Sample Inches", lastDistance);
	
	// get the new sample distance
	SmartDashboard::PutNumber("Entering Sample Number", measurementCount+1);
	InputDistance = -999.;
	SmartDashboard::PutNumber("New Sample Inches", InputDistance);
	//timer.Start();
	while (InputDistance <= 0.0) // wait for a distance to be entered
	{
		if(IsDisabled() || !IsOperatorControl()) // allow user to break out before any sample
		{
			printf("\n\nB R E A K I N G   I R   C A L I B R A T I O N   -   N O T   C O M P L E T E D\n\n");

			return;
		}

		//printf("%f, %d, %f\n", timer.Get(), __LINE__, InputDistance);
		InputDistance = SmartDashboard::GetNumber("New Sample Inches");
		Wait(.25);
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

	printf("\nSmoothed Voltage: %g  |  Measured Distance: %g\n", voltageSmoothed[measurementCount], ActualDistance[measurementCount]);

	// print statistics on the voltage samples for this distance
	sprintf(header, "Voltage for Distance Sample %d", measurementCount+1);
	voltageFuzzy.Print(stdout, header);

	IR.Push(voltageSmoothed[measurementCount], ActualDistance[measurementCount]);  // save the voltage and distance for the non-linear curve fits

	IRLinear.Push(voltageSmoothed[measurementCount], ActualDistance[measurementCount]);  // save the voltage and distance for the linear curve fit
	
	lastVoltage = voltageSmoothed[measurementCount]; // save this sample to display
	lastDistance = ActualDistance[measurementCount];
	}
	/***************************************************
	Completed all distance/voltage samples
	***************************************************/

	/***************************************************
	Compute least squares fits
	***************************************************/
    IR.Print(stdout, "non-linear least squares fits");
    IRLinear.Print(stdout, "linear (first-order) least squares fit");

    printf("\nsample#, average voltage, measured distance, power distance, logarithmic distance, exponential distance, linear distance\n");

    // make a table with the voltages of the distance samples and the measured and curve fit distances
    for (int measurementCount = 0; measurementCount < MAX_SAMPLES; measurementCount++)
	{
	PowerDistance[measurementCount] = IR.Power_A()*pow(voltageSmoothed[measurementCount], IR.Power_B());
	
	LogarithmicDistance[measurementCount] = IR.Logarithmic_A() + IR.Logarithmic_B()*log(voltageSmoothed[measurementCount]);
	
	ExponentialDistance[measurementCount] = IR.Exponential_A()*exp(IR.Exponential_B()*voltageSmoothed[measurementCount]);
	
	LinearDistance[measurementCount] = IRLinear.Intercept() + IRLinear.Slope()*voltageSmoothed[measurementCount];

	printf("%d, %g, %g, %g, %g, %g, %g\n", measurementCount+1, voltageSmoothed[measurementCount],
		ActualDistance[measurementCount], PowerDistance[measurementCount], LogarithmicDistance[measurementCount],
		ExponentialDistance[measurementCount], LinearDistance[measurementCount] );
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
	}
	
	printf("\n\nE N D I N G   I R   C A L I B R A T I O N   -   C O M P L E T E D\n\n");

	/**************************************************
	end test method
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
