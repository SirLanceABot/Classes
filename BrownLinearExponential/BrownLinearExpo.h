#ifndef SRC_BROWNLINEAREXPO_H_
#define SRC_BROWNLINEAREXPO_H_

class BrownLinearExpo
{
public:
	BrownLinearExpo();
	BrownLinearExpo(double f);
	void set_factor(double factor); // A higher scaling factor makes the filter more responsive to changes.
	void reset (double initial);
	double step(double measurement);
private:
	double estimate, double_smoothed, single_smoothed;
	double a;
};

#endif /* SRC_BROWNLINEAREXPO_H_ */
