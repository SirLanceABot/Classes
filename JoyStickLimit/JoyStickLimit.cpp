// Limit the rate of change of something such as JoyStick vector magnitude changes - x and y axis movements

#include <iostream>
#include <stdio.h>
#include <math.h>

int main() {

double x_prev = 0.;
double y_prev = 0.;
double x_new;
double y_new;
double length;
double angle;
double limit;

// test case 1
limit = .25;
x_new = -.7; // new joystick values
y_new = -.3;
length =sqrt( pow(x_prev - x_new, 2) + pow(y_prev - y_new, 2) );
if (length > limit) //requested too much change so limit it
    {
    printf("%f %f ", x_new, y_new);
    angle = atan2(x_prev - x_new, y_prev - y_new);
    x_new = x_prev - limit*sin(angle); // heading in the requested direction; use x_new, y_new for the motor speed
    y_new = y_prev - limit*cos(angle);
    printf("%f %f %f %f %f %f\n", x_prev, y_prev, x_new, y_new, angle, length);
    x_prev = x_new;
    y_prev = y_new;
    }

// test case 2
limit = .05;
x_new = .0; // new joystick values
y_new = .0;
length =sqrt( pow(x_prev - x_new, 2) + pow(y_prev - y_new, 2) );
if (length > limit) //requested too much change so limit it
    {
    printf("%f %f ", x_new, y_new);
    angle = atan2(x_prev - x_new, y_prev - y_new);
    x_new = x_prev - limit*sin(angle); // heading in the requested direction; use x_new, y_new for the motor speed
    y_new = y_prev - limit*cos(angle);
    printf("%f %f %f %f %f %f\n", x_prev, y_prev, x_new, y_new, angle, length);
    x_prev = x_new;
    y_prev = y_new;
    }

return 0;
}
/*
 with Limit of .25:
-0.700000 -0.300000 0.000000 0.000000 -0.229786 -0.098480 1.165905 0.761577
 with Limit of .05:
 0.000000 0.000000 -0.229786 -0.098480 -0.183829 -0.078784 -1.975688 0.250000
*/

