package TestJava;

public class RunningLeastSquares
{
	/*
	 3 non-linear least squares fits:

	 Power Law    y = Ax^B

	 Logarithmic  y = A + Blogx

	 Exponential y = Ae^Bx

	 Look at Wolfram Mathematica site for the equations already worked out
	 */

	public RunningLeastSquares()
	{
		Clear();
	}
	public final void Clear()
	{
		n = 0;
		S_LogxLogy = 0.0;
		S_LogxSqr = 0.0;
		S_LogySqr = 0.0;
		S_Logx = 0.0;
		S_Logy = 0.0;
		S_yLogx = 0.0;
		S_y = 0.0;
		S_xSqry = 0.0;
		S_yLogy = 0.0;
		S_xy = 0.0;
		S_xyLogy = 0.0;
	}
	public final void Push(double x, double y)
	{
		n++;
		S_LogxLogy += Math.log(x) * Math.log(y);
		S_LogxSqr += Math.pow(Math.log(x), 2);
		S_LogySqr += Math.pow(Math.log(y), 2);
		S_Logx += Math.log(x);
		S_Logy += Math.log(y);
		S_yLogx += y * Math.log(x);
		S_y += y;
		S_xSqry += Math.pow(x, 2) * y;
		S_yLogy += y * Math.log(y);
		S_xy += x * y;
		S_xyLogy += x * y * Math.log(y);
	}
	public final int NumDataValues()
	{
		return n;
	}
	public final double Power_A()
	{
		return Math.exp((S_Logy - Power_B() * S_Logx) / n);
	}
	public final double Power_B()
	{
		return (n * S_LogxLogy - S_Logx * S_Logy) / (n * S_LogxSqr - Math.pow(S_Logx, 2));
	}
	public final double Power_Correlation()
	{
		return Math.pow((n * S_LogxLogy - S_Logx * S_Logy), 2) / ((n * S_LogxSqr - Math.pow(S_Logx, 2)) * (n * S_LogySqr - Math.pow(S_Logy, 2)));
	}
	public final double Logarithmic_A()
	{
		return (S_y - Logarithmic_B() * S_Logx) / n;
	}
	public final double Logarithmic_B()
	{
		return (n * S_yLogx - S_y * S_Logx) / (n * S_LogxSqr - Math.pow(S_Logx, 2));
	}
	public final double Exponential_A()
	{
		// Regular fit equation gives greater weight to small values.
		// It is often better to weight points equally by minimizing SUM(y*(ln(y) - a - b*x)**2) which yields:
		return Math.exp((S_xSqry * S_yLogy - S_xy * S_xyLogy) / (S_y * S_xSqry - Math.pow(S_xy, 2)));
	}
	public final double Exponential_B()
	{
		// Regular fit equation gives greater weight to small values.
		// It is often better to weight points equally by minimizing SUM(y*(ln(y) - a - b*x)**2) which yields:
		return (S_y * S_xyLogy - S_xy * S_yLogy) / (S_y * S_xSqry - Math.pow(S_xy, 2));
	}
	public String toString()
	{	// for example:	System.out.print("Running Least Squares"); System.out.println(rls)
		String content;
		content =
				String.format("\nNumDataValues: %d\n", NumDataValues()) +
				String.format("y = Ax^B ==> y = %gx^%g   r^2 Correlation: %f\n", Power_A(), Power_B(), Power_Correlation()) +
				String.format("y = A + Blog(x) ==> y = %g %+gLog(x)\n", Logarithmic_A(), Logarithmic_B()) +
				String.format("y = Ae^Bx ==> y = %ge^%gx\n", Exponential_A(), Exponential_B());
		return content;
	}

	private int n;
	private double S_LogxLogy;
	private double S_Logx;
	private double S_Logy;
	private double S_yLogx;
	private double S_LogxSqr;
	private double S_LogySqr;
	private double S_y;
	private double S_xSqry;
	private double S_yLogy;
	private double S_xy;
	private double S_xyLogy;
}
