package StripChart;

public class Main
{
	public static void main(String args[])
	{
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

		StripChart myChart = new StripChart(30.0, -1.0, 1.0);

		for (int loopStartTime = 0; loopStartTime < 100000; loopStartTime++)
			System.out.println(
					myChart.PrintStripChart(loopStartTime, (loopStartTime / 1000.0 + 12.0) % 57.0, Math.sin(loopStartTime / 1000.0 + 12.0))
					);
	}
}