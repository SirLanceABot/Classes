#include "BrownLinearExpo.h"

BrownLinearExpo::BrownLinearExpo()
{
	a = 0.5;
	reset(0.);
}

BrownLinearExpo::BrownLinearExpo(double f)
{
	a = f;
	reset(0.);
}

void BrownLinearExpo::set_factor(double factor) // A higher scaling factor makes the filter more responsive to changes.
{
	a = factor;
}
void BrownLinearExpo::reset (double initial)
{
	single_smoothed = double_smoothed = estimate = initial;
}

double BrownLinearExpo::step(double measurement)
{
	single_smoothed = a * measurement + (1 - a) * single_smoothed;
	double_smoothed = a * single_smoothed + (1 - a) * double_smoothed;
	double est_a = (2*single_smoothed - double_smoothed);
	double est_b = (a / (1-a) )*(single_smoothed - double_smoothed);
	estimate = est_a + est_b;
	return estimate;
}
