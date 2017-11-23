package TestJava;

public class Main
{
	// sample IR proximity sensor voltages at various distances
	// get many voltage samples at each distance and average them.
	// compute curve fits of the measured data
	// graph results

	static private int AxisDistanceScale(double d)
	{
		return  Math.min(Math.max((int)(d * 2.0 + .5), 0), 100);
	}
	public static void main(String args[])
	{
		RunningLeastSquares IR = new RunningLeastSquares();
		RunningRegression IRLinear = new RunningRegression();
		RunningStats voltageFuzzy = new RunningStats();

		double voltageSample;
		double[] voltageSmoothed = new double[15];
		double[] ActualDistance = new double[15];
		double[] PowerDistance = new double[15];
		double[] LogarithmicDistance = new double[15];
		double[] ExponentialDistance = new double[15];
		double[] LinearDistance = new double[15];

		char[][] line = new char[101][101];

		double voltageFake = 0.3; // for simulation

		for (int measurementCount = 0; measurementCount < 15; measurementCount++) // distance/voltage measurements
		{
			voltageFake += .3; // for simulation

			voltageFuzzy.Clear(); // reset voltage average calculator

			for (int i = 0; i < 50; i++) // get 50 voltage samples at this distance and average them to quiet the noise
			{
				// get voltage sample
				// voltageFuzzy.Push(voltageSample); // add sample to average
				// wait(.02);  // wait a bit to take another sample voltage at this distance

				voltageSample = voltageFake + (Math.random() - .5) / 10.0; // +0.05 to -0.05  // for simulation using non-reproducible random - results vary run to run
				voltageFuzzy.Push(voltageSample); // for simulation - same statement as real
			}

			System.out.format("\nPROCESSING MEASUREMENT SAMPLE %d\n", measurementCount + 1);
			System.out.print(voltageFuzzy);

			voltageSmoothed[measurementCount] = voltageFuzzy.Mean(); // compute the average voltage at this distance

			ActualDistance[measurementCount] = 27.028 / 2.54 * Math.pow(voltageFake, -1.2259); // for simulation use spec sheet curve y = 27.028x^-1.2259  distance y function of voltage x
			//	ActualDistance[measurementCount] = 10. + 7.*log(voltageFake); // for simulation testing
			//	ActualDistance[measurementCount] = 10. * exp(.3*voltageFake); // for simulation testing
			//	ActualDistance[measurementCount] = 4. + 10. * voltageFake; // for simulation testing

			IR.Push(voltageSmoothed[measurementCount], ActualDistance[measurementCount]); // save the voltage and distance for the non-linear curve fits
			IRLinear.Push(voltageSmoothed[measurementCount], ActualDistance[measurementCount]); // save the voltage and distance for the linear curve fit

			System.out.printf("smoothed voltage: %g  ==>  measured distance: %g\n", voltageSmoothed[measurementCount], ActualDistance[measurementCount]);
		}

		System.out.print("\nNON-LINEAR LEAST SQUARES FITS\n");
		System.out.print(IR);

		System.out.print("\nLINEAR (FIRST-ORDER) LEAST SQUARES FIT\n");
		System.out.print(IRLinear);

		System.out.print("\nsample#, average voltage, measured distance, power distance, logarithmic distance, exponential distance, linear distance\n");

		// make a table with the voltages of the distance samples and the measured and curve fit distances
		for (int measurementCount = 0; measurementCount < 15; measurementCount++)
		{
			PowerDistance[measurementCount] = IR.Power_A() * Math.pow(voltageSmoothed[measurementCount], IR.Power_B());
			LogarithmicDistance[measurementCount] = IR.Logarithmic_A() + IR.Logarithmic_B() * Math.log(voltageSmoothed[measurementCount]);
			ExponentialDistance[measurementCount] = IR.Exponential_A() * Math.exp(IR.Exponential_B() * voltageSmoothed[measurementCount]);
			LinearDistance[measurementCount] = IRLinear.Intercept() + IRLinear.Slope() * voltageSmoothed[measurementCount];

			System.out.printf("%d, %g, %g, %g, %g, %g, %g\n", measurementCount + 1, voltageSmoothed[measurementCount], ActualDistance[measurementCount], PowerDistance[measurementCount], LogarithmicDistance[measurementCount], ExponentialDistance[measurementCount], LinearDistance[measurementCount]);
		}

		// initialize the graphic presentation
		for (int axisDistance = 0; axisDistance <= 100; axisDistance++)
		{
			for (int axisVoltage = 0; axisVoltage <= 100; axisVoltage++)
			{
				line[axisDistance][axisVoltage] = axisVoltage % 2 == 0 ? '.': ' ';
			}
		}

		// locate the points on the graph
		for (int measurementCount = 0; measurementCount < 15; measurementCount++)
		{
			int axisVoltage = Math.min(Math.max((int)(voltageSmoothed[measurementCount] * 20.0 + .5), 0), 100);

			line[AxisDistanceScale(LinearDistance [measurementCount])][axisVoltage] = '1';

			line[AxisDistanceScale(LogarithmicDistance[measurementCount])][axisVoltage] = 'L';

			line[AxisDistanceScale(ExponentialDistance[measurementCount])][axisVoltage] = 'E';

			line[AxisDistanceScale(PowerDistance [measurementCount])][axisVoltage] = 'P';

			line[AxisDistanceScale(ActualDistance [measurementCount])][axisVoltage] = 'M';
		}

		// print graph
		for (int axisDistance = 100; axisDistance >= 0; axisDistance--)
			System.out.println(line[axisDistance]);

	}
}
/* C++
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

/* Java


PROCESSING MEASUREMENT SAMPLE 1

NumDataValues:     50
Mean:              0.594279
Variance:          0.000972
StandardDeviation: 0.031178
Skewness:          0.175630
Kurtosis:          -1.335798
Maximum:           0.647459
Minimum:           0.550559
smoothed voltage: 0.594279  ==>  measured distance: 19.9042

PROCESSING MEASUREMENT SAMPLE 2

NumDataValues:     50
Mean:              0.899308
Variance:          0.000914
StandardDeviation: 0.030229
Skewness:          -0.036342
Kurtosis:          -1.303713
Maximum:           0.948259
Minimum:           0.851243
smoothed voltage: 0.899308  ==>  measured distance: 12.1081

PROCESSING MEASUREMENT SAMPLE 3

NumDataValues:     50
Mean:              1.199482
Variance:          0.000795
StandardDeviation: 0.028197
Skewness:          0.179119
Kurtosis:          -1.228100
Maximum:           1.249231
Minimum:           1.150779
smoothed voltage: 1.19948  ==>  measured distance: 8.50965

PROCESSING MEASUREMENT SAMPLE 4

NumDataValues:     50
Mean:              1.498365
Variance:          0.000800
StandardDeviation: 0.028289
Skewness:          0.035612
Kurtosis:          -0.968135
Maximum:           1.549530
Minimum:           1.451972
smoothed voltage: 1.49837  ==>  measured distance: 6.47306

PROCESSING MEASUREMENT SAMPLE 5

NumDataValues:     50
Mean:              1.795784
Variance:          0.000709
StandardDeviation: 0.026619
Skewness:          0.292505
Kurtosis:          -1.026944
Maximum:           1.849257
Minimum:           1.754807
smoothed voltage: 1.79578  ==>  measured distance: 5.17656

PROCESSING MEASUREMENT SAMPLE 6

NumDataValues:     50
Mean:              2.103525
Variance:          0.000711
StandardDeviation: 0.026657
Skewness:          -0.135563
Kurtosis:          -0.689804
Maximum:           2.149835
Minimum:           2.051470
smoothed voltage: 2.10352  ==>  measured distance: 4.28520

PROCESSING MEASUREMENT SAMPLE 7

NumDataValues:     50
Mean:              2.407607
Variance:          0.000556
StandardDeviation: 0.023570
Skewness:          -0.155794
Kurtosis:          -0.610030
Maximum:           2.446198
Minimum:           2.352573
smoothed voltage: 2.40761  ==>  measured distance: 3.63814

PROCESSING MEASUREMENT SAMPLE 8

NumDataValues:     50
Mean:              2.690296
Variance:          0.000831
StandardDeviation: 0.028834
Skewness:          0.352411
Kurtosis:          -0.918266
Maximum:           2.746887
Minimum:           2.650038
smoothed voltage: 2.69030  ==>  measured distance: 3.14899

PROCESSING MEASUREMENT SAMPLE 9

NumDataValues:     50
Mean:              2.999652
Variance:          0.000932
StandardDeviation: 0.030531
Skewness:          -0.203515
Kurtosis:          -1.198228
Maximum:           3.048165
Minimum:           2.950707
smoothed voltage: 2.99965  ==>  measured distance: 2.76743

PROCESSING MEASUREMENT SAMPLE 10

NumDataValues:     50
Mean:              3.299739
Variance:          0.000951
StandardDeviation: 0.030832
Skewness:          0.041058
Kurtosis:          -1.201504
Maximum:           3.348366
Minimum:           3.250397
smoothed voltage: 3.29974  ==>  measured distance: 2.46226

PROCESSING MEASUREMENT SAMPLE 11

NumDataValues:     50
Mean:              3.598907
Variance:          0.000716
StandardDeviation: 0.026749
Skewness:          0.140448
Kurtosis:          -1.153305
Maximum:           3.647515
Minimum:           3.555619
smoothed voltage: 3.59891  ==>  measured distance: 2.21314

PROCESSING MEASUREMENT SAMPLE 12

NumDataValues:     50
Mean:              3.897019
Variance:          0.000825
StandardDeviation: 0.028726
Skewness:          0.014809
Kurtosis:          -1.291756
Maximum:           3.945875
Minimum:           3.851255
smoothed voltage: 3.89702  ==>  measured distance: 2.00629

PROCESSING MEASUREMENT SAMPLE 13

NumDataValues:     50
Mean:              4.199492
Variance:          0.000771
StandardDeviation: 0.027760
Skewness:          -0.072573
Kurtosis:          -1.278710
Maximum:           4.241665
Minimum:           4.150900
smoothed voltage: 4.19949  ==>  measured distance: 1.83206

PROCESSING MEASUREMENT SAMPLE 14

NumDataValues:     50
Mean:              4.496851
Variance:          0.000783
StandardDeviation: 0.027982
Skewness:          0.076119
Kurtosis:          -1.164197
Maximum:           4.549918
Minimum:           4.450137
smoothed voltage: 4.49685  ==>  measured distance: 1.68348

PROCESSING MEASUREMENT SAMPLE 15

NumDataValues:     50
Mean:              4.805717
Variance:          0.000853
StandardDeviation: 0.029209
Skewness:          -0.158458
Kurtosis:          -1.237411
Maximum:           4.849709
Minimum:           4.753833
smoothed voltage: 4.80572  ==>  measured distance: 1.55542

NON-LINEAR LEAST SQUARES FITS

NumDataValues: 15
y = Ax^B ==> y = 10.6003x^-1.22269   r^2 Correlation: 0.999986
y = A + Blog(x) ==> y = 11.5205 -7.54412Log(x)
y = Ae^Bx ==> y = 22.6328e^-0.665728x

LINEAR (FIRST-ORDER) LEAST SQUARES FIT

NumDataValues: 15
y = A + Bx ==> y = 13.3969 -3.04277x
Slope:           -3.042775
Intercept:       13.396919
r^2 Correlation: -0.812918

sample#, average voltage, measured distance, power distance, logarithmic distance, exponential distance, linear distance
1, 0.594279, 19.9042, 20.0288, 15.4465, 15.2377, 11.5887
2, 0.899308, 12.1081, 12.0690, 12.3212, 12.4374, 10.6605
3, 1.19948, 8.50965, 8.48656, 10.1483, 10.1845, 9.74716
4, 1.49837, 6.47306, 6.46533, 8.46987, 8.34695, 8.83773
5, 1.79578, 5.17656, 5.18135, 7.10388, 6.84759, 7.93275
6, 2.10352, 4.28520, 4.27024, 5.91060, 5.57908, 6.99637
7, 2.40761, 3.63814, 3.62040, 4.89200, 4.55665, 6.07111
8, 2.69030, 3.14899, 3.16086, 4.05447, 3.77498, 5.21096
9, 2.99965, 2.76743, 2.76699, 3.23333, 3.07237, 4.26965
10, 3.29974, 2.46226, 2.46251, 2.51402, 2.51600, 3.35656
11, 3.59891, 2.21314, 2.21459, 1.85929, 2.06165, 2.44626
12, 3.89702, 2.00629, 2.00925, 1.25891, 1.69054, 1.53917
13, 4.19949, 1.83206, 1.83375, 0.694977, 1.38221, 0.618810
14, 4.49685, 1.68348, 1.68660, 0.178856, 1.13396, -0.285986
15, 4.80572, 1.55542, 1.55503, -0.322291, 0.923208, -1.22579
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
