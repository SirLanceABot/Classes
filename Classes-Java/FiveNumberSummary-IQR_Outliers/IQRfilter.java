/*
Five Number Summary of a set of data and Interquartile Range (IQR) based outliers

Quantiles are the set of values/points that divides the dataset into groups of equal size.

if 4 parts, then they are quartiles
In descriptive statistics, the interquartile range (IQR) tells you the spread of the middle half of
your distribution.

The quartiles of a ranked set of data values are three points which divide the data into exactly four
equal parts, each part comprising one quarter of the amount of data.

Q1 is defined as the middle value between the smallest value and the median of the data set.
Q2 is the median of the data.
Q3 is the middle value between the median and the highest value of the data set.

The interquartile range, IQR, tells us the range where the bulk of the values lie.
The interquartile range is calculated by subtracting the first quartile from the third quartile. 
IQR = Q3 - Q1

https://levelup.gitconnected.com/how-to-easily-forecast-the-stock-price-probabilities-time-series-using-iqr-interquartile-range-9d97dafd5a32
https://en.wikipedia.org/wiki/Interquartile_range

Uses 
1. Unlike range, IQR tells where the majority of data lies and is thus preferred over range. 
2. IQR can be used to identify outliers in a data set (values that are too far out of the IQR). 
3. Gives the central tendency of the data. 

Example: 
Data : 1, 19, 7, 6, 5, 9, 12, 27, 18, 2, 15
The data set after being sorted is 
1, 2, 5, 6, 7, 9, 12, 15, 18, 19, 27
As mentioned above Q2 is the median of the data. 
Hence Q2 = 9
Q1 is the median of lower half, taking Q2 as pivot.
So Q1 = 5
Q3 is the median of upper half talking Q2 as pivot. 
So Q3 = 18
Therefore IQR for given data = Q3-Q1 = 18-5 = 13
*/

/*
 * based on github jbosstm  narayana
 * 
 * JBoss, Home of Professional Open Source.
 * Copyright 2014 Red Hat, Inc., and individual contributors
 * as indicated by the @author tags. See the copyright.txt file in the
 * distribution for a full listing of individual contributors.
 *
 * This is free software; you can redistribute it and/or modify it
 * under the terms of the GNU Lesser General Public License as
 * published by the Free Software Foundation; either version 2.1 of
 * the License, or (at your option) any later version.
 *
 * This software is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public
 * License along with this software; if not, write to the Free
 * Software Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA, or see the FSF site: http://www.fsf.org.
 */

 import java.util.ArrayList;
 import java.util.Arrays;
 import java.util.stream.Collectors;
 
/**
 * Class Usage:
 * 
 * <p>Compute the Five Number Summary:
 * <p>             IQRfilter.fiveNumberSummary(data)
 * 
 * <p>Find outliers in data:
 * <p>             IQRfilter.findOutliers(data, outlierRangeMultiplier)
 * 
 * <p>Remove outliers in time series data:
 * <p>             IQRfilter smoother = new IQRfilter(windowSize, threshold);
 * <p>             smoother.calculate(element));
 * 
 * <p>Other methods available:
 * <p>             getQ1()
 * <p>             getMedian()
 * <p>             getAverage()
 * <p>             getQ3()
 * <p>             getIQR()
 * <p>             dumpFnsIqr()
 * <p>             dumpData()
 * <p>             main()
 * <p>
 * <p>This class needs to be instantiated only for the running window outlier/smoother function.
 * <Methods for FNS and IQR calculations are static thus no instantiation needed.
 */

public class IQRfilter {

    private int n; // window size - number of elements in the running window
    private double[] y; // window data storage
    private double threshold; // IQR multiplier to set outlier threshold

    /**
     * 
     * @param WindowSize 4 is a good min; more like 9 would be slow to accept a step change and maybe quicker to find a spike
     * @param threshold 1.5 is typical; 3.0 rejects only extreme outliers
     */
    public IQRfilter(int WindowSize, double threshold)
    {
        this.n = WindowSize;
        this.threshold = threshold;
        this.y = new double[WindowSize]; // y here will be the sensor data history [0] the oldest
        for (int i = 0; i < this.y.length; i++) this.y[i] = Double.NaN;
    }

    /**
     * An outlier is any data point typically more than 1.5 interquartile ranges (IQRs)
     * below the first quartile or above the third quartile.
     * 
     * <p>This method determines if the most recent point is a data outlier based on IQR of the
     * current window of data and returns either the original value if not an outlier or the 3rd
     * or 1st quartile if an outlier. "Rejected" data is still retained for use in the next window.
     * 
     * <p>Thus it's a sort of spike remover or smoothing method.
     * 
     * <p>This is a little different than typical implementations that return a set of outliers
     * from the set of all data points. The result of selecting outliers from the entire set
     * of points uses all the data for the determination. Checking only the last point for
     * outlier does not take advantage of future data that could result in a different outcome.
     *
     * @param value given data point 
     * @return given data point or 1st or 3rd quartile value if point is an outlier
     */
    public double calculate(double value)
    {
        double yPredictedIQR = value;// assume value is good to return; change later if it's an outlier
        // shove all data down in history to make room for the current value
        // could use WPILIB circular buffer but this is easy and efficient
        for (int k = 0; k < n-1; k++) {
            y[k] = y[k+1];
        }
        y[n-1] = value; // stuff the current data
        
        var tempY = Arrays.copyOf(y, y.length); // all this quartile stuff sorts the data destroying the original sequence

        if(y[0] != Double.NaN) // check oldest to see if it's been set yet indicating buffer now filled
        { // buffer is filled so start using the history
            double[] fns = fiveNumberSummary(tempY);

            // System.out.printf("min(q0)=%f q1=%f median(q2)=%f q3=%f max(q4)=%f IQR=%f%n",
            // fns[0], fns[1], fns[2], fns[3], fns[4], getIQR(fns));

            double iqr = getIQR(fns); // positive since data sorted ascending
            double outlierBoundary = threshold * iqr;
            double lb = fns[1] - outlierBoundary;
            double ub = fns[3] + outlierBoundary;
    
            if (value < lb) // is the latest point a small outlier?
                yPredictedIQR = fns[1]; // replace with q1
            else
            if (value > ub) // is the latest point a large outlier?
                yPredictedIQR = fns[3]; // replace with q3
        }
        // System.out.println(value + "  " + yPredictedIQR);
        return yPredictedIQR;
    }

    /**
     * q1 value
     * @param values
     * @return q1 value
     */
    static double getQ1(double[] values) {
        int sz = values.length;

        if (sz == 1)
            return values[0];
        else if (sz % 2 == 0)
            return getMedian(values, 0, sz / 2 - 1);
        else
            return getMedian(values, 0, sz / 2 - 1);
    }

    /**
     * Median value between two elements of the data array
     * @param values
     * @param from
     * @param to
     * @return
     */
    static double getMedian(double[] values, int from, int to) {
        int sz = to - from + 1;

        if (sz % 2 == 0)
             return (values[from + sz/2] + values[from + (sz/2) - 1] ) / 2;
        else
            return values[from + (sz / 2)];
    }

     /**
      * Average getter
      * @param values
      * @return average of values
      */
      static double getAverage(double[] values) {
        double tot = 0;

        for (double d : values)
            tot += d;

        return tot / values.length;
    }

    /**
     * q3 value
     * @param values
     * @return q3 value
     */
    static double getQ3(double[] values) {
        int sz = values.length;

        if (sz == 1)
            return values[0];
        else if (sz % 2 == 0)
            return getMedian(values, sz / 2, sz - 1);
        else
            return getMedian(values, sz / 2 + 1, sz - 1);
    }

    /**
     * Test main - example usage of five number summary of data and IQR based outliers
     * @param args - not used
     */

    /**
     * Five Number Summary of the data
     * @param values - input data but MODIFIED - returned in ascending sorted order
     * @return min(q0), q1, median(q2), q3, max(q4) [the FNS array]
     */
    static double[] fiveNumberSummary(double[] values)
    {
        Arrays.sort(values); // using quartiles requires rank order
        double median = getMedian(values, 0, values.length - 1);
        double q1 = getQ1(values);
        double q3 = getQ3(values);

        return new double[]{values[0], q1, median, q3, values[values.length-1]};
    }

    /**
     * Find data outliers based on IQR
     * An outlier is any data point typically more than 1.5 interquartile ranges (IQRs)
     * below the first quartile or above the third quartile.
     *
     * CAUTION - input data modified upon return
     *
     * @param values - input data but MODIFIED - returned in ascending sorted order
     * @param outlierRangeMultiplier - criterion multiplies IQR
     *          typical values are 1.5=>outlier 3.0=>extreme outlier
     * @return array of outliers of the input values based on input param outlierRangeMultiplier
     */
    static double[] findOutliers(double[] values, double outlierRangeMultiplier) {
        ArrayList<Double> outlierList = new ArrayList<Double>();
        double[] fns = fiveNumberSummary(values);
        double iqr = getIQR(fns);
        double outlierBoundary = outlierRangeMultiplier * iqr;
        double lb = fns[1] - outlierBoundary;
        double ub = fns[3] + outlierBoundary;

        for(var value:values)
        {
            if (value < lb || value > ub)
                outlierList.add(value);
        }
        double[] outliers = outlierList.stream().mapToDouble(i -> i).toArray();
        return outliers;
    }

    /**
     * Interquartile range getter
     * @param fns Five Number Summary array
     * @return IQR of FNS, q3-q1
     */
    static double getIQR(double[] fns) {
    return fns[3] - fns[1];
    }

    /**
     * Format for display a Five Number Summary with Interquartile Range
     * @param fns Five Number Summary array
     * @return formatted string
     */
    static String dumpFnsIqr(double[] fns) {
    return String.format("Five Number Summary:[min(q0)=%f, q1=%f, median(q2)=%f, q3=%f, max(q4)=%f]%nInterquartile Range=%f",
        fns[0], fns[1], fns[2], fns[3], fns[4], getIQR(fns));
    }

    /**
     * Format for display a double array
     * @param array
     * @return formatted string
     */
    static String dumpData(double[] array) {
    return Arrays.stream(array)
        .mapToObj(i -> String.format("%f", i))
        .collect(Collectors.joining(", ", "[", "]"));
    }

    public static void main(String[] args) {
 
        /**
        * Test main - example usage of five number summary of data and IQR based outliers
        * @param args - not used
        */
        double[][] tests = {
          { 1., 19., 7., 6., 5., 9., 12., 27., 18., 2., 15. },
          { 1., 200.0, 5.0, 6.0, 9.0, 12.0, -200.0 },
          { 12.0, 9.0, 6.0, 5.0, 2.0, 26.0 },
          { 1., 3., 4., 5., 5., 6., 7., 11. },
          { 2. },
          { 20., 25. },
          { 20., 25., 42.},
          { 90., 87., 80., 65.}
        };
      
        // run all tests - batches of data   
        for (double[] test : tests) {
            // Input test data
            System.out.println("\n\nInput:" + IQRfilter.dumpData(test));

            // Sorted data and FiveNumberSummary of the data and IQR
            var fns = IQRfilter.fiveNumberSummary(test);
            System.out.println(IQRfilter.dumpFnsIqr(fns));

            // IQR outliers
            var outliers = IQRfilter.findOutliers(test, 1.5);
            System.out.println("IQR Outliers:" + IQRfilter.dumpData(outliers));
        }

        // run a test simulating a series of real-time stream of data
        int windowSize = 6;
        double threshold = 1.5;
        IQRfilter smoother = new IQRfilter(windowSize, threshold);
        for (double[] test : tests)
        for (double element : test)
        {
            System.out.println(element + " -> " + smoother.calculate(element));
        }
    }
}
/*

Input:[1.000000, 19.000000, 7.000000, 6.000000, 5.000000, 9.000000, 12.000000, 27.000000, 18.000000, 2.000000, 15.000000]
Five Number Summary:[min(q0)=1.000000, q1=5.000000, median(q2)=9.000000, q3=18.000000, max(q4)=27.000000]
Interquartile Range=13.000000
IQR Outliers:[]


Input:[1.000000, 200.000000, 5.000000, 6.000000, 9.000000, 12.000000, -200.000000]
Five Number Summary:[min(q0)=-200.000000, q1=1.000000, median(q2)=6.000000, q3=12.000000, max(q4)=200.000000]
Interquartile Range=11.000000
IQR Outliers:[-200.000000, 200.000000]


Input:[12.000000, 9.000000, 6.000000, 5.000000, 2.000000, 26.000000]
Five Number Summary:[min(q0)=2.000000, q1=5.000000, median(q2)=7.500000, q3=12.000000, max(q4)=26.000000]
Interquartile Range=7.000000
IQR Outliers:[26.000000]


Input:[1.000000, 3.000000, 4.000000, 5.000000, 5.000000, 6.000000, 7.000000, 11.000000]
Five Number Summary:[min(q0)=1.000000, q1=3.500000, median(q2)=5.000000, q3=6.500000, max(q4)=11.000000]
Interquartile Range=3.000000
IQR Outliers:[]


Input:[2.000000]
Five Number Summary:[min(q0)=2.000000, q1=2.000000, median(q2)=2.000000, q3=2.000000, max(q4)=2.000000]
Interquartile Range=0.000000
IQR Outliers:[]


Input:[20.000000, 25.000000]
Five Number Summary:[min(q0)=20.000000, q1=20.000000, median(q2)=22.500000, q3=25.000000, max(q4)=25.000000]
Interquartile Range=5.000000
IQR Outliers:[]


Input:[20.000000, 25.000000, 42.000000]
Five Number Summary:[min(q0)=20.000000, q1=20.000000, median(q2)=25.000000, q3=42.000000, max(q4)=42.000000]
Interquartile Range=22.000000
IQR Outliers:[]


Input:[90.000000, 87.000000, 80.000000, 65.000000]
Five Number Summary:[min(q0)=65.000000, q1=72.500000, median(q2)=83.500000, q3=88.500000, max(q4)=90.000000]
Interquartile Range=16.000000
IQR Outliers:[]
1.0 -> 1.0
2.0 -> 2.0
5.0 -> 5.0
6.0 -> 6.0
7.0 -> 7.0
9.0 -> 9.0
12.0 -> 12.0
15.0 -> 15.0
18.0 -> 18.0
19.0 -> 19.0
27.0 -> 27.0
-200.0 -> 12.0
1.0 -> 1.0
5.0 -> 5.0
6.0 -> 6.0
9.0 -> 9.0
12.0 -> 12.0
200.0 -> 12.0
2.0 -> 2.0
5.0 -> 5.0
6.0 -> 6.0
9.0 -> 9.0
12.0 -> 12.0
26.0 -> 12.0
1.0 -> 1.0
3.0 -> 3.0
4.0 -> 4.0
5.0 -> 5.0
5.0 -> 5.0
6.0 -> 6.0
7.0 -> 7.0
11.0 -> 7.0
2.0 -> 2.0
20.0 -> 20.0
25.0 -> 25.0
20.0 -> 20.0
25.0 -> 25.0
42.0 -> 25.0
65.0 -> 65.0
80.0 -> 80.0
87.0 -> 87.0
90.0 -> 90.0
 */