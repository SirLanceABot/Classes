package frc.robot;
// https://en.wikipedia.org/wiki/Alpha_beta_filter
// https://www.mstarlabs.com/control/engspeed.html

// Key steps for initializing an alpha-beta filter:
// Set initial state:
// Position (x): Assign the initial position of the object you are tracking based on your best available information.
// Velocity (v): Assign the initial velocity of the object, which could be zero if you assume it is initially stationary.
// Choose alpha and beta gains:
// Alpha (α): This gain determines how much the current measurement influences the position estimate. A larger alpha means faster response to new measurements but can also lead to more noise.
// Beta (β): This gain controls how much the current measurement impacts the velocity estimate. A larger beta means more emphasis on recent velocity changes. 



public class AlphaBetaFilter {

	double dt = 0;
    double alpha = 0;
    double beta = 0;
    double prevX = 0;
    double prevV = 0;

    public AlphaBetaFilter(double alpha, double beta, double dt) {
        this(alpha, beta, dt, 0., 0.);
    }

    public AlphaBetaFilter(double alpha, double beta, double dt, double xInitial, double vInitial) {
        this.alpha = alpha;
        this.beta = beta;
        this.dt = dt;
        this.prevX = xInitial;
        this.prevV = vInitial;
    }

    public double calculate(double measurement) {
        // is this initializing good or bad or unnecessary?
        // if (prevX == 0 && prevV == 0) {
        //     prevX = measurement;
        //     return measurement;
        // }
		double predictedX = prevX + prevV * dt;// angular position prediction xp  <--  x + (ΔT) v
		double predictedV = prevV;

		double residual = measurement - predictedX;

		predictedX = predictedX + alpha * residual;// adjusted angular position estimate x+  <--  xp + α (xmeas - xp)

        predictedV = predictedV + (beta / dt) * residual;// adjusted velocity estimate v+  <--  v + β (xmeas - xp)

		prevX = predictedX;
		prevV = predictedV;

        return predictedX;
	}

    public static void main(String[] args) {
        double[] measurements = {10.0, 11.5, 12.0, 13.5, 14.0, 15.5};

        AlphaBetaFilter filter = new AlphaBetaFilter(0.8, 0.2, 0.1, measurements[0], 0.);

        for (double measurement : measurements) {
            double predictedValue = filter.calculate(measurement);
            System.out.println("Measurement: " + measurement + ", Predicted: " + predictedValue);
        }
    }
/*
Measurement: 10.0, Predicted: 10.0
Measurement: 11.5, Predicted: 11.2
Measurement: 12.0, Predicted: 11.9
Measurement: 13.5, Predicted: 13.26
Measurement: 14.0, Predicted: 13.98
Measurement: 15.5, Predicted: 15.328
*/
}
