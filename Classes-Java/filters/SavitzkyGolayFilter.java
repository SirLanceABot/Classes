package frc.robot;

public class SavitzkyGolayFilter {

// Predicts current value from least squares fit of window of history. No other rejection criterion.

/*
  Compute the Savitzky-Golay least squares filtering coefficients assuming sampling at a fixed time interval
  
  Based on ideas and output verified with:
  https://gregstanleyandassociates.com/whitepapers/FaultDiagnosis/Filtering/LeastSquares-Filter/leastsquares-filter.htm
  
  which has a good explanation of usage of this and many other filters.

  The intention is to use a window of old data (including the newest value) to form a least squares linear
  fitting line that is used to predict what the newest value would be based on (mostly) old data. That
  predicted value based on history can be compared to the actual value. Beware the filter overshoots step changes.

  The least squares formula for a straight line is derived in many math sources and not repeated here.
  The formula for a straight line with two parameters a and b:
        y = a + bx
  with a and b calculated from "n" data points (x sub k, y sub k)

  a = ((Sum y)(Sum x^2) - (Sum x)(Sum xy))/(n(Sum x^2) - (Sum x)^2)

  b = (n(Sum xy) - (Sum x)(Sum y))/(n(Sum x^2) - (Sum x)^2)

  Assuming the data are spaced equally (the x direction or "t" time for sampled data), any arbitrary values
  for x sub k can be used.  Select 0 for the newest value, -1 for the second newest, and so on back to -(n-1)
  for the oldest value.

  The numerical values for y are not yet known (to be measured later) so express them as known but symbolically
  as y(0), y(-1), y(-2), ..., y(-(n-1)).

  Since for filtering, only the newest value at time = x = 0 is needed from the least square fit and that is
  the y-intercept or the "a" value and no calculation is needed using the "b" value.  (Plug a value of x = 0
  in the "b" formula and it's b = 0.)

  Note that for smoothing, the midpoint of the window is normally used and not the newest endpoint so the
  formulation for smoothing is similar but again the time is arbitrary so the midpoint is called 0 to
  simplify calculations and the oldest data is at -(n-1)/2 and the newest at (n-1)/2.

  calculated coefficients for oldest data y[0] to newest data y[n-1]
2  0.00000  1.00000
3 -0.16667  0.33333  0.83333
4 -0.20000  0.10000  0.40000  0.70000
5 -0.20000  0.00000  0.20000  0.40000  0.60000
6 -0.19048 -0.04762  0.09524  0.23810  0.38095  0.52381
7 -0.17857 -0.07143  0.03571  0.14286  0.25000  0.35714  0.46429
8 -0.16667 -0.08333  0.00000  0.08333  0.16667  0.25000  0.33333  0.41667
9 -0.15556 -0.08889 -0.02222  0.04444  0.11111  0.17778  0.24444  0.31111  0.37778
10 -0.14545 -0.09091 -0.03636  0.01818  0.07273  0.12727  0.18182  0.23636  0.29091  0.34545
11 -0.13636 -0.09091 -0.04545  0.00000  0.04545  0.09091  0.13636  0.18182  0.22727  0.27273  0.31818
12 -0.12821 -0.08974 -0.05128 -0.01282  0.02564  0.06410  0.10256  0.14103  0.17949  0.21795  0.25641  0.29487
13 -0.12088 -0.08791 -0.05495 -0.02198  0.01099  0.04396  0.07692  0.10989  0.14286  0.17582  0.20879  0.24176  0.27473

Example:
If using 4 data points, the linear Savitzky-Golay filter is

      y(k filtered) = .7 y(k) + .4 y(k-1) + .1 y(k-2) - .2 y(k-3)

where
y(k) is the newest (current) input
y(k-1) is the previous input
y(k-2) is the next oldest input
y(k-3) is the oldest input used for the 4-point filter
y(k filtered) is the newest (current) filtered (output) value.

In the nomenclature of the Java method and array and reversing the order of the presentation terms
so [0] is always the oldest and [n-1] is always the newest

      y_filtered = coef[0]*y[0] + coef[1]*y[1] + coef[2]*y[2] + coef[3]*y[3]

Then compare the y[n-1] to the y filtered.

Note that I disagree somewhat with the nomenclature used by Greg Stanley since it conflicts with
the standard usage of the x and y axes for linear least squares.  I get that they use a standard
of y = f(x) but that is confusing when the x is actually the y of a preceding equation.  Watch out!
*/
private int WindowSize;
private double[] y;
private double[] coef;
private boolean bufferNotFilled = true;
private int bufferedCountIndex = -1;

public SavitzkyGolayFilter(int WindowSize)
{
    this.WindowSize = WindowSize;
    this.y = new double[WindowSize]; // y here will be the sensor data history [0] the oldest
    this.coef = LeastSquaresFiltering(WindowSize); 
}

public double calculate(double value)
{
    // shove all data down in history to make room for the current value
    // could use WPILIB circular buffer but this is easy and efficient
    for (int k = 0; k < WindowSize-1; k++) {
        y[k] = y[k+1];
      }
    y[WindowSize-1] = value; // stuff the current data

    // if the buffer hasn't filled yet then return the input value - no filtering yet
    if(bufferNotFilled)
    {
        bufferedCountIndex++;
        if(bufferedCountIndex+1 < WindowSize) return value;// WindowSize starts at 1; bufferCountIndex starts at 0
        bufferNotFilled = false;
    }
     
    // use the precalculated coefficients of the least square filter with the historical data
    double yPredictedSG = 0;
    for(int k =0; k < WindowSize; k++) {
        //System.out.print(coef[k] + " ");
        yPredictedSG += coef[k]*y[k]; 
    }  
    return yPredictedSG;
}

    static public double[] LeastSquaresFiltering (int WindowSize) {
    double[] x = new double[WindowSize]; // x here is pseudo time at regularly spaced intervals
    double[] coef = new double[WindowSize];
    double xSum=0, xSqrSum=0;
  
    for (int k = 0; k < WindowSize; k++) {
      x[k] = k - (WindowSize - 1); // generate the pseudo time; oldest = -(n-1) to newest = 0
      xSum += x[k];
      xSqrSum += x[k]*x[k];
    }
    double xSumSqr = xSum*xSum;
    for (int k = 0; k < WindowSize; k++) {
      coef[WindowSize-1-k] = (xSqrSum - xSum*x[WindowSize-1-k]) / (WindowSize*xSqrSum - xSumSqr);
    }
  
    return coef;
  }
    
}
