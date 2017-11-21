#include "StripChart.h"

int main()
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

    StripChart myChart(30., -1., 1.);

    for (unsigned long loopStartTime =0; loopStartTime <100000; loopStartTime++)
    {
	std::cout << "\n";
	myChart.PrintStripChart(loopStartTime, fmod(loopStartTime/1000.+12., 57.), sin(loopStartTime/1000.+12.));
    }
    return 0;
}
