package frc.robot;

import java.util.*;
 
class MADFilter
{
// Predicts current value from window of history and requested rejection criterion.

/*
Median Absolute Deviation (MAD) of an array of numbers

Median Absolute Deviation is a robust way to identify outliers.
It replaces standard deviation or variance with median deviation
and the mean with the median. The result is a method that isn’t
as affected by outliers as using the mean and standard deviation.

https://eurekastatistics.com/using-the-median-absolute-deviation-to-find-outliers
https://stats.stackexchange.com/questions/123895/mad-formula-for-outlier-detection
https://medium.com/@joaopedroferrazrodrigues/outliers-make-us-go-mad-univariate-outlier-detection-b3a72f1ea8c7

x~ is the median of the sample
MAD is the absolute difference between the median and each point in the sample
MAD = median{|xi - x~|}
modified Z-score Mi = 0.6745(xi - x~)/MAD
rule of thumb is |Mi| > 3.5 is an outlier (or |Mi| > 3.0)
0.6745 is the 0.75th quartile of the standard normal distribution, to which the MAD converges
*/

    private int n;
    private double[] y;
    private double[] fns;
    private double mad;
    private double threshold; 

    /**
     * 
     * @param WindowSize 4 is a good min; more like 9 would be slow to accept a step change and maybe quicker to find a spike
     * @param threshold 3.5 is typical; sometimes 3.0
     */
    public MADFilter(int WindowSize, double threshold)
    {
        this.n = WindowSize;
        this.threshold = threshold;
        this.y = new double[WindowSize]; // y here will be the sensor data history [0] the oldest
        for (int i = 0; i < this.y.length; i++) this.y[i] = Double.NaN;
    }

    /**
     * An outlier is any data point typically with a modified Z-score > 3.5.
     * 
     * This method determines if the most recent point is a data outlier based on MAD
     * and returns either the original value if not an outlier or the 3rd or 1st quartile
     * if an outlier.
     * 
     * Thus is sort of like a spike detector or somewhat of a smoothing method.
     * 
     * This is a little different than typical implementations that return a set of outliers
     * from the set of all data points. The result of selecting outliers from the entire set
     * of points uses all the data for the determination. Checking only the last point for
     * outlier does not take advantage of future data that could result in a different outcome.
     *
     * @param value given data point 
     * @return given data point or 1st or 3rd quartile value if point is an outlier
     */
    public double calculate(double value)
    {
        double yPredictedMAD = value;// assume value is good to return; change later if it's an outlier
        // shove all data down in history to make room for the current value
        // could use WPILIB circular buffer but this is easy and efficient
        for (int k = 0; k < n-1; k++) {
            y[k] = y[k+1];
        }
        y[n-1] = value; // stuff the current data
        
        var tempY = Arrays.copyOf(y, y.length); // all this quartile stuff sorts the data destroying the original sequence

        fns = IQRFilter.fiveNumberSummary(tempY); // min(q0), q1, median(q2), q3, max(q4)

        if(y[0] != Double.NaN) // check oldest to see if it's been set yet indicating buffer now filled
        { // buffer is filled so start using the history
            var median =fns[2];
            mad = mad(tempY);

            if((0.6745*(value - median)/mad) < threshold) // is the latest point a small outlier?
                yPredictedMAD = fns[1]; // replace with q1
            else
            if((0.6745*(value - median)/mad) > threshold) // is the latest point a large outlier?
                yPredictedMAD = fns[3]; // replace with q3
        }

        return yPredictedMAD;
    }

    /**
     * Function for calculating the mean of an array of numbers.
     * @param data the array of numbers
     * @return mean
     */
    public double mean(double data[])
    {
        double sum = 0;
        for (int i = 0; i < n; i++)
            sum += data[i];
 
        return sum / (double)n;
    }
 
    /**
     * Function for calculating the median of an array of numbers.
     * @param data the array of numbers
     * @return median
     */
    public double median(double data[])
    {
        // sort the array
        Arrays.sort(data);
 
        // check for odd case
        if (n % 2 != 0)
            return data[n / 2];
 
        return (data[n/2 - 1] + data[n/2]) / 2.0;
    }

    /**
     * Function for calculating the Median Absolute Deviation (MAD) of an array of numbers.
     * @param data the array of numbers
     * @return MAD
     */
    public double mad(final double[] data) {
        var median =fns[2];
        double[] deviationSum = new double[n];
        for (int i = 0; i < n; i++) {
            deviationSum[i] = Math.abs(median - data[i]);
        }
        mad = median(deviationSum);
        return mad;
    }

    /**
     * test example
     * @param args
     */
    public void main(String args[])
    {
        // double data[] = {1, 2, 3, 3, 4, 4, 4, 5, 5.5, 6, 6, 6.5, 7, 7, 7.5, 8, 9, 12, -52, 90}; // test data
        // double mean = mean(data);
        // double median = median(data);
        // double mad = mad(data);
        
        // System.out.println("Mean = " + mean);
        // System.out.println("Median = " + median);
        // System.out.println("MAD = " + mad);

        // Arrays.stream(data)
        //     .peek(point -> System.out.println(point))
        //     .filter(point->outlier(point))
        //     .forEach(point->System.out.println("  is an outlier"));
    }
}

/* Output
Mean = 6.925
Median = 5.75
MAD = 1.75   
-52.0
  is an outlier
1.0
2.0
3.0
3.0
4.0
4.0
4.0
5.0
5.5
6.0
6.0
6.5
7.0
7.0
7.5
8.0
9.0
12.0
90.0
  is an outlier
*/