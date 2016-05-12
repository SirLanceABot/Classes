// Limit the rate of change of something such as the motor controller power

// Started but not finished - not even compilable as is.
// obsolete if using CAN Talon since that has power limit built-in

#ifndef PowerChangeLimit_H_
#define PowerChangeLimit_H_

class PowerChangeLimit{
public:
    PowerChangeLimit();
    PowerChangeLimit PowerChangeLimited(PowerChangeLimit);
    PowerChangeLimit operator+(const PowerChangeLimit a, const PowerChangeLimit b);
    PowerChangeLimit& operator+=(const PowerChangeLimit &rhs);
    static PowerChangeLimit min(const PowerChangeLimit a, const PowerChangeLimit b);
    static PowerChangeLimit abs(const PowerChangeLimit a, const PowerChangeLimit b);
    setPreviousPower(double, double);

private:
    double x_prev;
    double y_prev;
}


#endif


PowerChangeLimit::PowerChangeLimit()
{
    setPreviousPower(0., 0.)
}

PowerChangeLimit::setPreviousPower(double x, double y)
{
    x_prev = x;
    y_prev = y;
}

PowerChangeLimit PowerChangeLimit::abs(PowerChangeLimit a) static
{
    a.x_prev = absf(x;
    a.y_prev = absf(y;
}