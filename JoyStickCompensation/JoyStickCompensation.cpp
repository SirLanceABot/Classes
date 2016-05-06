#include "JoyStickCompensation.h"
float JoyStickCompensation(float X, int curve, float scale)
{
/*
 * Change the response of the JoyStick
 * Select a response curve to change the feel of the stick
 * Reverse axis by using a negative scale factor
 * Limit or expand action with scale factor but output clamped to curve range
 * JoyStick input any value (nominally -1 to 1)
 * Compensation curve number 1 to 3; Output is 0 if bad curve number
*/

#define tableMin 0
#define tableMax 12
#define curvesMin 1
#define curvesMax 3

float curvesX[tableMax] =
/*  In */   {0.f, 0.03f, 0.1f, 0.2f,  0.3f,  0.4f,  0.5f,  0.6f,  0.7f,  0.8f,  0.9f,  1.f};
float curvesY[curvesMax][tableMax] =
{ /* must be >= 0 not decreasing values */
/*  1 */    {0.f,  0.f, 0.06f, 0.13f, 0.25f, 0.37f, 0.48f, 0.58f, 0.67f, 0.75f, 0.88f, 1.f},
/*  2 */    {0.f,  0.f, 0.06f, 0.14f, 0.24f, 0.35f, 0.47f, 0.60f, 0.74f, 0.87f, 0.96f, 1.f},
/*  3 */    {0.f,  0.f, 0.01f, 0.04f, 0.09f, 0.16f, 0.25f, 0.36f, 0.49f, 0.64f, 0.81f, 1.f},
};
int i;
float interpolateY;

float aX = X; // save input to use the sign
if (X < 0.f) X = -X;  // absolute value of x since - and + are symmetric

if (curve > curvesMax || curve < curvesMin) return 0.f;
curve = curve - 1;

if (X < curvesX[tableMin])
    {interpolateY = curvesY[curve][tableMin];} // below table
else if (X > curvesX[tableMax-1])
    {interpolateY = curvesY[curve][tableMax-1];} // above table
else
    { // within table, find out where
    for (i = tableMin+1; i <= tableMax-2; i++)
        {if (X < curvesX[i]) break;}
    interpolateY =  curvesY[curve][i-1] + (curvesY[curve][i] - curvesY[curve][i-1])*((X -curvesX[i-1])/(curvesX[i] - curvesX[i-1]));
    }
/* scale, clamp, and retain the sign relationship */
interpolateY = scale*interpolateY;
if      (interpolateY < -curvesY[curve][tableMax-1]) interpolateY = -curvesY[curve][tableMax-1];
else if (interpolateY >  curvesY[curve][tableMax-1]) interpolateY =  curvesY[curve][tableMax-1];
if (aX < 0.f) interpolateY = -interpolateY;
return interpolateY;
}
