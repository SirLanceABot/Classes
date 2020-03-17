package stripchart;

/*
StripChart myChart(
    setpoint,		// center of left process variable graph
    controllerMin,	// minimum value of right controller graph
    controllerMax); 	// maximum value of right controller graph

myChart.PrintStripChart(
    loopStartTime,	// time in milliseconds
    input,		// process variable - input to controller
    output);		// output from controller
*/

public class StripChart
{
	private double Strip_Minimum_1;
	private double Strip_Center_1;
	private double Strip_Maximum_1;
	private double Strip_Minimum_2;
	private double Strip_Maximum_2;

	// left chart auto scales around the given center (argument 1)
	// right chart is fixed by the given minimum and given maximum (arguments 2 and 3)
	public StripChart(double aStrip_Center_1, double aStrip_Minimum_2, double aStrip_Maximum_2)
	{
		this.Strip_Center_1 = aStrip_Center_1;
		this.Strip_Minimum_2 = aStrip_Minimum_2;
		this.Strip_Maximum_2 = aStrip_Maximum_2;
		Strip_Maximum_1 = -Double.MAX_VALUE;
		Strip_Minimum_1 = Double.MAX_VALUE;
		System.out.print("[Time msecs] [PV chart min, value, max] ~~~~~~~~~ [Controller chart, min, value, max]\n");
	}

	// millisecond time value (argument 1)
	// left and right chart data (arguments 2 and 3)
	public final String PrintStripChart(long Time, double Strip_Data_1, double Strip_Data_2)
	{
		StringBuilder Strip_Graph_1 = new StringBuilder(" . . . . . + . . . . . ");
		StringBuilder Strip_Graph_2 = new StringBuilder(" . . . . . + . . . . . ");
		int Strip_Normalized_1;
		int Strip_Normalized_2;
		double Strip_Span_1;
		double Strip_MaxSpan_1;
		double Strip_MinSpan_1;
		double Strip_Span_2;
		double Strip_Center_2;

		// graphing Strip_Data_1 =========================================================================
		if (Strip_Maximum_1 < Strip_Data_1)
		{
			Strip_Maximum_1 = Strip_Data_1;
		}
		if (Strip_Minimum_1 > Strip_Data_1)
		{
			Strip_Minimum_1 = Strip_Data_1;
		}

		Strip_MaxSpan_1 = Strip_Maximum_1 - Strip_Center_1;
		Strip_MinSpan_1 = Strip_Center_1 - Strip_Minimum_1;

		Strip_Span_1 = 2.0 * (Math.abs(Strip_MaxSpan_1) > Math.abs(Strip_MinSpan_1) ? Math.abs(Strip_MaxSpan_1) : Math.abs(Strip_MinSpan_1));
		if (Strip_Span_1 == 0)
		{
			Strip_Span_1 = 1.0; // handle first point
		}
		Strip_Normalized_1 = (int)(((Strip_Data_1 - Strip_Center_1) > 0 ? .25 : -.25) + 21.0 * (Strip_Data_1 - Strip_Center_1) / Strip_Span_1);
		if (Strip_Normalized_1 < -10)
		{
			Strip_Graph_1.setCharAt( 0, 'X'); // not used if auto max/min to the data
		}
		else if (Strip_Normalized_1 > +10)
		{
			Strip_Graph_1.setCharAt( 22, 'X');
		}
		else
		{
			Strip_Graph_1.setCharAt( 11 + Strip_Normalized_1, '|');
		}
		// end graphing Strip_Data_1 =====================================================================

		// graphing Strip_Data_2 ========================================================================
		Strip_Span_2 = Strip_Maximum_2 - Strip_Minimum_2;
		Strip_Center_2 = Strip_Minimum_2 + Strip_Span_2 / 2.0;
		Strip_Normalized_2 = (int)(((Strip_Data_2 - Strip_Center_2) > 0 ? .25 : -.25) + 21.0 * (Strip_Data_2 - Strip_Center_2) / Strip_Span_2);
		if (Strip_Normalized_2 < -10)
		{
			Strip_Graph_2.setCharAt( 0, 'X');
		}
		else if (Strip_Normalized_2 > +10)
		{
			Strip_Graph_2.setCharAt( 22, 'X');
		}
		else
		{
			Strip_Graph_2.setCharAt( 11 + Strip_Normalized_2, '|');
		}
		// end graphing Strip_Data_2 ====================================================================

		// print the graphs =======================================================================

		return	String.format("%7d %8.3f %8.3f %8.3f \t %s~%s %8.3f %8.3f %8.3f",
				Time, Strip_Center_1 - (Strip_Span_1 / 2.0), Strip_Data_1, Strip_Center_1 + (Strip_Span_1 / 2.0),
				Strip_Graph_1, Strip_Graph_2, Strip_Minimum_2, Strip_Data_2, Strip_Maximum_2);

		// end print the graphs ===================================================================

	}
}