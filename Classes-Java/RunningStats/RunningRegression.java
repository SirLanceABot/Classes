package TestJava;

//RunningStats.cpp

///#include "RunningStats.h"
///#include <cmath>
///#include <vector>

//http://www.johndcook.com/running_regression.html

// RunningRegression.h

/*
Computing linear regression in one pass
The RunningRegression class is the analog of the RunningStats class described here and uses that class.
You add pairs of (x, y) values by using the Push. At any point along the way you can call the Slope,
Intercept, or Correlation functions to see the current value of these statistics.

You can also combine two RunningRegression objects by using the + and += operators. For example, you might
accrue data on several different threads in parallel then add their RunningRegression objects together.
 */

///#include "RunningStats.h"

public class RunningRegression
{

	//http://www.johndcook.com/running_regression.html

	// RunningRegression.h

	/*
	Computing linear regression in one pass
	The RunningRegression class is the analog of the RunningStats class described here and uses that class.
	You add pairs of (x, y) values by using the Push. At any point along the way you can call the Slope,
	Intercept, or Correlation functions to see the current value of these statistics.

	You can also combine two RunningRegression objects by using the add and copyFrom methods. For example, you might
	accrue data on several different threads in parallel then add their RunningRegression objects together.
	 */

	public RunningRegression()
	{
		Clear();
	}
	public final void Clear()
	{
		x_stats.Clear();
		y_stats.Clear();
		S_xy = 0.0;
		n = 0;
	}
	public final void copyFrom(RunningRegression r)
	{
		x_stats.copyFrom(r.x_stats);
		y_stats.copyFrom(r.y_stats);
		S_xy = r.S_xy;
		n = r.n;
	}
	public final void Push(double x, double y)
	{
		S_xy += (x_stats.Mean() - x) * (y_stats.Mean() - y) * (double)n / (double)(n + 1);
		x_stats.Push(x);
		y_stats.Push(y);
		n++;
	}
	public final int NumDataValues()
	{
		return n;
	}
	public final double Slope()
	{
		double S_xx = x_stats.Variance() * (n - 1.0);
		return S_xy / S_xx;
	}
	public final double Intercept()
	{
		return y_stats.Mean() - Slope() * x_stats.Mean();
	}
	public final double Correlation()
	{
		double t = x_stats.StandardDeviation() * y_stats.StandardDeviation();
		return S_xy / ((n - 1) * t);
	}
	public String toString()
	{	// for example:	System.out.print("Running Regression"); System.out.println(rr)
		String content;
		content =
				String.format("\nNumDataValues: %d\n", NumDataValues()) +
				String.format("y = A + Bx ==> y = %g %+gx\n", Intercept(), Slope()) +
				String.format("Slope:           %f\n", Slope()) +
				String.format("Intercept:       %f\n", Intercept()) +
				String.format("r^2 Correlation: %f\n", Correlation());
		return content;
	}
	public RunningRegression add (RunningRegression b)
	{
		RunningRegression combined = new RunningRegression();
		combined.x_stats.copyFrom(this.x_stats.add(b.x_stats));
		combined.y_stats.copyFrom(this.x_stats.add(b.y_stats));
		combined.n = this.n + b.n;
		double delta_x = b.x_stats.Mean() - this.x_stats.Mean();
		double delta_y = b.y_stats.Mean() - this.y_stats.Mean();
		combined.S_xy = this.S_xy + b.S_xy + (double)(this.n * b.n) * delta_x * delta_y / (double)combined.n;
		return combined;
	}

	private RunningStats x_stats = new RunningStats();
	private RunningStats y_stats = new RunningStats();
	private double S_xy;
	private int n;
}