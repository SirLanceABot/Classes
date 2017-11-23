package TestJava;

//This file includes both the Running Stats and Running Regression routines

// http://www.johndcook.com/skewness_kurtosis.html

//RunningStats.h

/*
Computing skewness and kurtosis in one pass
The code is an extension of the method of Knuth and Welford for computing standard deviation
in one pass through the data. It computes skewness and kurtosis as well with a similar interface.
In addition to only requiring one pass through the data, the algorithm is numerically stable and accurate.

To use the code, create a RunningStats object and use the Push method to insert data values. After
accruing your data, you can obtain sample statistics by calling one of these methods:

�Mean
�Variance
�StandardDeviation
�Skewness
�Kurtosis
You can also combine two RunningStats objects by using the + and += operators. For example, you might
accrue data on several different threads in parallel then add their RunningStats objects together to
create a single object with the state that it would have had if all the data had been accumulated by it
alone.
References:

1.Wikipedia
2.Timothy B. Terriberry. Computing Higher-Order Moments Online.
3.Philippe P�bay. SANDIA REPORT SAND2008-6212 (2008). Formulas for Robust, One-Pass Parallel Computation
 of Co- variances and Arbitrary-Order Statistical Moments.

 */

/*  EXAMPLE
		RunningStats sumit; // construct statistics object
		RunningRegression aLine; // construct regression object

		System.out.printf("%d, %f\n",i, xm); // print data point - assumes xm was set above here
		sumit.Push(xm); // add data point to statistical routine
		aLine.Push(xm, 1.); // add data point to regression routine - this example assumes data are flat line at 1 (y = 1.) otherwise enter a y value

		System.out.print("Raw Signal"); // print statistics
		System.out.print(sumit); // print statistics
		System.out.print("raw signal regression line vs 1."); // print regression
		System.out.print(aline); // print regression
 */

public class RunningStats
{

	//This file includes both the Running Stats and Running Regression routines

	// http://www.johndcook.com/skewness_kurtosis.html

	//RunningStats.h

	/*
	Computing skewness and kurtosis in one pass
	The code is an extension of the method of Knuth and Welford for computing standard deviation
	in one pass through the data. It computes skewness and kurtosis as well with a similar interface.
	In addition to only requiring one pass through the data, the algorithm is numerically stable and accurate.

	To use the code, create a RunningStats object and use the Push method to insert data values. After
	accruing your data, you can obtain sample statistics by calling one of these methods:

	�Mean
	�Variance
	�StandardDeviation
	�Skewness
	�Kurtosis
	You can also combine two RunningStats objects by using the + and += operators. For example, you might
	accrue data on several different threads in parallel then add their RunningStats objects together to
	create a single object with the state that it would have had if all the data had been accumulated by it
	alone.
	References:

	1.Wikipedia
	2.Timothy B. Terriberry. Computing Higher-Order Moments Online.
	3.Philippe P�bay. SANDIA REPORT SAND2008-6212 (2008). Formulas for Robust, One-Pass Parallel Computation
	 of Co- variances and Arbitrary-Order Statistical Moments.

	 */

	/*  EXAMPLE
			#include <stats_running.h>

			RunningStats sumit;        // construct statistics object
			RunningRegression aLine;   // construct regression object

			fprintf(stdout, "%d, %f\n",i, xm);  // print data point - assumes xm was set above here
			sumit.Push(xm);            // add data point to statistical routine
			aLine.Push(xm, 1.);        // add data point to regression routine - this example assumes data are flat line at 1 (y = 1.) otherwise enter a y value

			sumit.Print(stdout, "Raw Signal");  // print statistics
			aLine.Print(stdout, "raw signal regression line vs 1.");  // print regression
	 */

	public RunningStats()
	{
		Clear();
	}
	public final void Clear()
	{
		n = 0;
		M1 = M2 = M3 = M4 = 0.0;
		maxdata = -Double.MAX_VALUE;
		mindata = Double.MAX_VALUE;
	}
	public final void copyFrom(RunningStats r)
	{
		n = r.n;
		M1 = r.M1;
		M2 = r.M2;
		M3 = r.M3;
		M4 = r.M4;
		maxdata = r.maxdata;
		mindata = r.mindata;
	}
	public final void Push(double x)
	{
		double delta;
		double delta_n;
		double delta_n2;
		double term1;
		int n1 = n;
		n++;
		delta = x - M1;
		delta_n = delta / n;
		delta_n2 = delta_n * delta_n;
		term1 = delta * delta_n * n1;
		M1 += delta_n;
		M4 += term1 * delta_n2 * (n * n - 3 * n + 3) + 6 * delta_n2 * M2 - 4 * delta_n * M3;
		M3 += term1 * delta_n * (n - 2) - 3 * delta_n * M2;
		M2 += term1;
		if (maxdata < x)
		{
			maxdata = x;
		}
		if (mindata > x)
		{
			mindata = x;
		}
	}
	public final int NumDataValues()
	{
		return n;
	}
	public final double Mean()
	{
		return M1;
	}
	public final double Variance()
	{
		return M2 / (n - 1.0);
	}
	public final double StandardDeviation()
	{
		return Math.sqrt(Variance());
	}
	public final double Skewness()
	{
		return Math.sqrt((double)n) * M3 / Math.pow(M2, 1.5);
	}
	public final double Kurtosis()
	{
		return (double)n * M4 / (M2 * M2) - 3.0;
	}
	public final double Maximum()
	{
		return maxdata;
	}
	public final double Minimum()
	{
		return mindata;
	}
	public String toString()
	{	// for example:	System.out.print("Running Stats"); System.out.println(rs)
		String content;
		content =
				String.format("\nNumDataValues:     %d\n", NumDataValues()) +
				String.format("Mean:              %f\n", Mean()) +
				String.format("Variance:          %f\n", Variance()) +
				String.format("StandardDeviation: %f\n", StandardDeviation()) +
				String.format("Skewness:          %f\n", Skewness()) +
				String.format("Kurtosis:          %f\n", Kurtosis()) +
				String.format("Maximum:           %f\n", Maximum()) +
				String.format("Minimum:           %f\n", Minimum());
		return content;
	}
	public RunningStats add (RunningStats b)
	{
		RunningStats combined = new RunningStats();
		combined.n = this.n + b.n;
		double delta = b.M1 - this.M1;
		double delta2 = delta * delta;
		double delta3 = delta * delta2;
		double delta4 = delta2 * delta2;
		combined.M1 = (this.n * this.M1 + b.n * b.M1) / combined.n;
		combined.M2 = this.M2 + b.M2 + delta2 * this.n * b.n / combined.n;
		combined.M3 = this.M3 + b.M3 + delta3 * this.n * b.n * (this.n - b.n) / (combined.n * combined.n);
		combined.M3 += 3.0 * delta * (this.n * b.M2 - b.n * this.M2) / combined.n;
		combined.M4 = this.M4 + b.M4 + delta4 * this.n * b.n * (this.n * this.n - this.n * b.n + b.n * b.n) / (combined.n * combined.n * combined.n);
		combined.M4 += 6.0 * delta2 * (this.n * this.n * b.M2 + b.n * b.n * this.M2) / (combined.n * combined.n) + 4.0 * delta * (this.n * b.M3 - b.n * this.M3) / combined.n;
		combined.maxdata = this.maxdata > b.maxdata ? this.maxdata : b.maxdata;
		combined.mindata = this.mindata < b.mindata ? this.mindata : b.mindata;
		return combined;
	}

	private int n;
	private double M1;
	private double M2;
	private double M3;
	private double M4;
	private double maxdata;
	private double mindata;
}