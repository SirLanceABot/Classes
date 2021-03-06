// sample IR proximity sensor voltages at various distances
// get many voltage samples at each distance and average them.
// compute curve fits of the measured data
// graph results

#include <math.h>
#include "stats_running.h"
#include <stdlib.h>
#include <iostream>

int main()
    {
    RunningLeastSquares IR;
    RunningRegression IRLinear;
    RunningStats voltageFuzzy;

    double voltageSample;
    double voltageSmoothed[15];
    double ActualDistance[15];
    double PowerDistance[15];
    double LogarithmicDistance[15];
    double ExponentialDistance[15];
    double LinearDistance[15];

    char header[200];
    char line[101][101];

    double voltageFake = 0.3;  // for simulation
    srand(12345); //random number seed for simulation

    for (int measurementCount = 0; measurementCount < 15; measurementCount++) // distance/voltage measurements
	{
	voltageFake+=.3;  // for simulation

	voltageFuzzy.Clear(); // reset voltage average calculator

	for (int i=0; i <50; i++) // get 50 voltage samples at this distance and average them to quiet the noise
	    {
	    // get voltage sample
	    // voltageFuzzy.Push(voltageSample); // add sample to average
	    // wait(.02);  // wait a bit to take another sample voltage at this distance

	    voltageSample = voltageFake + (((double)rand()/RAND_MAX) - .5) / 10.; // +0.05 to -0.05  // for simulation
	    voltageFuzzy.Push(voltageSample); // for simulation - same statement as real
	    }

	sprintf(header, "processing measurement sample %d", measurementCount+1);

	voltageFuzzy.Print(stdout, header);  // print statistics on the voltage samples for this distance

	voltageSmoothed[measurementCount] = voltageFuzzy.Mean();  // compute the average voltage at this distance

	ActualDistance[measurementCount] = 27.028/2.54 * pow(voltageFake, -1.2259); // for simulation use spec sheet curve y = 27.028x^-1.2259  distance y function of voltage x
//	ActualDistance[measurementCount] = 10. + 7.*log(voltageFake); // for simulation testing
//	ActualDistance[measurementCount] = 10. * exp(.3*voltageFake); // for simulation testing
//	ActualDistance[measurementCount] = 4. + 10. * voltageFake; // for simulation testing

	IR.Push(voltageSmoothed[measurementCount], ActualDistance[measurementCount]);  // save the voltage and distance for the non-linear curve fits
	IRLinear.Push(voltageSmoothed[measurementCount], ActualDistance[measurementCount]);  // save the voltage and distance for the linear curve fit

	printf("smoothed voltage: %g  |  measured distance: %g\n", voltageSmoothed[measurementCount], ActualDistance[measurementCount]);
	}

    IR.Print(stdout, "non-linear least squares fits");
    IRLinear.Print(stdout, "linear (first-order) least squares fit");

    printf("\nsample#, average voltage, measured distance, power distance, logarithmic distance, exponential distance, linear distance\n");

    // make a table with the voltages of the distance samples and the measured and curve fit distances
    for (int measurementCount = 0; measurementCount < 15; measurementCount++)
	{
	PowerDistance[measurementCount] = IR.Power_A()*pow(voltageSmoothed[measurementCount], IR.Power_B());
	LogarithmicDistance[measurementCount] = IR.Logarithmic_A() +  IR.Logarithmic_B()*log(voltageSmoothed[measurementCount]);
	ExponentialDistance[measurementCount] = IR.Exponential_A()*exp(IR.Exponential_B()*voltageSmoothed[measurementCount]);
	LinearDistance[measurementCount] = IRLinear.Intercept() + IRLinear.Slope()*voltageSmoothed[measurementCount];

	printf("%d, %g, %g, %g, %g, %g, %g\n", measurementCount+1, voltageSmoothed[measurementCount],
		ActualDistance[measurementCount], PowerDistance[measurementCount], LogarithmicDistance[measurementCount],
		ExponentialDistance[measurementCount], LinearDistance[measurementCount] );
	}

    // initialize the graphic presentation
    for (int axisDistance = 0; axisDistance <= 100; axisDistance++)
	for (int axisVoltage = 0; axisVoltage <= 100; axisVoltage++)
	    line[axisDistance][axisVoltage] = axisVoltage%2 == 0 ? '.': ' ';

    // locate the points on the graph
	for (int measurementCount = 0; measurementCount < 15; measurementCount++)
	    {
	    int axisVoltage = std::min(std::max((int)(voltageSmoothed[measurementCount]*20. + .5), 0), 100);

		#define AxisDistanceScale(d) std::min(std::max((int)(d*2. + .5), 0), 100)

		line[AxisDistanceScale(LinearDistance     [measurementCount])][axisVoltage] = '1';
	    
		line[AxisDistanceScale(LogarithmicDistance[measurementCount])][axisVoltage] = 'L';
	    
		line[AxisDistanceScale(ExponentialDistance[measurementCount])][axisVoltage] = 'E';
	    
		line[AxisDistanceScale(PowerDistance      [measurementCount])][axisVoltage] = 'P';
	    
		line[AxisDistanceScale(ActualDistance     [measurementCount])][axisVoltage] = 'M';
	    }

    // print graph
    for (int axisDistance = 100; axisDistance >= 0; axisDistance--)
	{
	printf("%101.101s\n", line[axisDistance]);
	}

    return 0;
    }
/*
processing measurement sample 1
NumDataValues:     50
Mean:              0.597247
Variance:          0.000722
StandardDeviation: 0.026877
Skewness:          0.005897
Kurtosis:          -1.204589
Maximum:           0.646957
Minimum:           0.554053
smoothed voltage: 0.597247  |  measured distance: 19.9042

processing measurement sample 2
NumDataValues:     50
Mean:              0.897183
Variance:          0.000919
StandardDeviation: 0.030314
Skewness:          0.020490
Kurtosis:          -1.411316
Maximum:           0.947467
Minimum:           0.851617
smoothed voltage: 0.897183  |  measured distance: 12.1081

processing measurement sample 3
NumDataValues:     50
Mean:              1.198798
Variance:          0.000846
StandardDeviation: 0.029092
Skewness:          0.146725
Kurtosis:          -1.184870
Maximum:           1.249683
Minimum:           1.151968
smoothed voltage: 1.1988  |  measured distance: 8.50965

processing measurement sample 4
NumDataValues:     50
Mean:              1.500976
Variance:          0.000978
StandardDeviation: 0.031275
Skewness:          0.030007
Kurtosis:          -1.361228
Maximum:           1.546817
Minimum:           1.450818
smoothed voltage: 1.50098  |  measured distance: 6.47306

processing measurement sample 5
NumDataValues:     50
Mean:              1.798750
Variance:          0.000772
StandardDeviation: 0.027788
Skewness:          -0.007807
Kurtosis:          -0.963389
Maximum:           1.849396
Minimum:           1.751312
smoothed voltage: 1.79875  |  measured distance: 5.17656

processing measurement sample 6
NumDataValues:     50
Mean:              2.101145
Variance:          0.000928
StandardDeviation: 0.030462
Skewness:          -0.071706
Kurtosis:          -1.319655
Maximum:           2.149631
Minimum:           2.051016
smoothed voltage: 2.10114  |  measured distance: 4.2852

processing measurement sample 7
NumDataValues:     50
Mean:              2.398645
Variance:          0.000950
StandardDeviation: 0.030820
Skewness:          -0.002606
Kurtosis:          -1.233924
Maximum:           2.449762
Minimum:           2.351730
smoothed voltage: 2.39864  |  measured distance: 3.63814

processing measurement sample 8
NumDataValues:     50
Mean:              2.697447
Variance:          0.000989
StandardDeviation: 0.031454
Skewness:          0.037255
Kurtosis:          -1.412407
Maximum:           2.747253
Minimum:           2.650772
smoothed voltage: 2.69745  |  measured distance: 3.14899

processing measurement sample 9
NumDataValues:     50
Mean:              2.999743
Variance:          0.000632
StandardDeviation: 0.025133
Skewness:          0.284953
Kurtosis:          -0.696754
Maximum:           3.049744
Minimum:           2.953485
smoothed voltage: 2.99974  |  measured distance: 2.76743

processing measurement sample 10
NumDataValues:     50
Mean:              3.298543
Variance:          0.000834
StandardDeviation: 0.028879
Skewness:          0.102054
Kurtosis:          -1.150590
Maximum:           3.348611
Minimum:           3.253345
smoothed voltage: 3.29854  |  measured distance: 2.46226

processing measurement sample 11
NumDataValues:     50
Mean:              3.601534
Variance:          0.000924
StandardDeviation: 0.030398
Skewness:          -0.051422
Kurtosis:          -1.203990
Maximum:           3.649530
Minimum:           3.551205
smoothed voltage: 3.60153  |  measured distance: 2.21314

processing measurement sample 12
NumDataValues:     50
Mean:              3.898589
Variance:          0.000781
StandardDeviation: 0.027942
Skewness:          -0.098122
Kurtosis:          -1.319478
Maximum:           3.947650
Minimum:           3.854443
smoothed voltage: 3.89859  |  measured distance: 2.00629

processing measurement sample 13
NumDataValues:     50
Mean:              4.203392
Variance:          0.000781
StandardDeviation: 0.027940
Skewness:          0.162534
Kurtosis:          -1.246389
Maximum:           4.249393
Minimum:           4.161145
smoothed voltage: 4.20339  |  measured distance: 1.83206

processing measurement sample 14
NumDataValues:     50
Mean:              4.504674
Variance:          0.000756
StandardDeviation: 0.027497
Skewness:          -0.098514
Kurtosis:          -0.950884
Maximum:           4.548541
Minimum:           4.453207
smoothed voltage: 4.50467  |  measured distance: 1.68348

processing measurement sample 15
NumDataValues:     50
Mean:              4.804252
Variance:          0.000767
StandardDeviation: 0.027687
Skewness:          -0.339284
Kurtosis:          -0.938177
Maximum:           4.847842
Minimum:           4.753122
smoothed voltage: 4.80425  |  measured distance: 1.55542

non-linear least squares fits
NumDataValues: 15
y = Ax^B ==> y = 10.612x^-1.22338   r^2 Correlation: 0.999998

y = A + Blog(x) ==> y = 11.5234 -7.54376Log(x)
y = Ae^Bx ==> y = 22.6343e^-0.665448x

linear (first-order) least squares fit
NumDataValues: 15
y = A + Bx ==> y = 13.3901 -3.03915x
Slope:           -3.039148
Intercept:       13.390149
r^2 Correlation: -0.812505

sample#, average voltage, measured distance, power distance, logarithmic distance, exponential distanc

1, 0.597247, 19.9042, 19.9363, 15.4117, 15.2112, 11.575
2, 0.897183, 12.1081, 12.1183, 12.3419, 12.4589, 10.6635
3, 1.1988, 8.50965, 8.50079, 10.1556, 10.1933, 9.74682
4, 1.50098, 6.47306, 6.45689, 8.45979, 8.33651, 8.82846
5, 1.79875, 5.17656, 5.17451, 7.09455, 6.83798, 7.92348
6, 2.10114, 4.2852, 4.27867, 5.92232, 5.59159, 7.00446
7, 2.39864, 3.63814, 3.63875, 4.92336, 4.58731, 6.10031
8, 2.69745, 3.14899, 3.15192, 4.03771, 3.76014, 5.19221
9, 2.99974, 2.76743, 2.76783, 3.23641, 3.07497, 4.27349
10, 3.29854, 2.46226, 2.46428, 2.52009, 2.5205, 3.36539
11, 3.60153, 2.21314, 2.21309, 1.85716, 2.06026, 2.44455
12, 3.89859, 2.00629, 2.00859, 1.25928, 1.69073, 1.54176
13, 4.20339, 1.83206, 1.83187, 0.691403, 1.38034, 0.615417
14, 4.50467, 1.68348, 1.68312, 0.169195, 1.12957, -0.300225
15, 4.80425, 1.55542, 1.55563, -0.316515, 0.925415, -1.21068
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . M . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . L . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . E . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . E . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . M . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . 1 . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . 1 . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . E . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . 1 . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . 1 . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . M . . E . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . 1 . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . E . . 1 . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . M . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . L . . 1 . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . E . . . . . . . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . M . . . . . L . . 1 . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . M . . E . . . . . 1 . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . E . . . . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . M . . . . . . . . 1 . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . M . . M . . . . . . . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . M . . 1 . . . . . . . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . M . . M . . M . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . E . . E . . M . . M . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . E . . E . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . L . . . . . . . .
. . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . . L . . L . .
*/
