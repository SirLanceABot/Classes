/*
Five Number Summary of a set of data and Interquartile Range (IQR) based outliers

Quantiles are the set of values/points that divides the dataset into groups of equal size.

If 4 groups, then they are quartiles.
In descriptive statistics, the interquartile range (IQR) tells you the spread of the middle half of
your distribution.

The quartiles of a ranked set of data values are three points which divide the data into exactly four
equal parts, each part comprising one quarter of the amount of data.

Q1 is defined as the middle value between the smallest value and the median of the data set.
Q2 is the median of the data.
Q3 is the middle value between the median and the highest value of the data set.
Note that for an odd number of values the median value is used twice - once for Q1 and for Q3. This
is the inclusive method. (If the exclusive method is used and the number of values is odd, the
median value is not used - the values adjacent to the median are used.)
Note that for an even number of values the median value of the complete set of values is the average
of the two central values but that is not used for the IQR calculation. The two central values are
considered the medians for determining Q1 and Q3.
For this program consider the minimum value as Q0 and the maximum value as Q4 (thus 5 numbers).

The interquartile range, IQR, tells us the range where the bulk of the values lie.
The interquartile range is calculated by subtracting the first quartile from the third quartile. 
IQR = Q3 - Q1

Uses 
1. Unlike range, IQR tells where the majority of data lies and is thus may be preferred over range.
2. IQR can be used to identify outliers in a data set (values that are too far out of the IQR). 
3. Gives the central tendency of the data. 

Example: 
Data : 1, 19, 7, 6, 5, 9, 12, 27, 18, 2, 15
The data set after being sorted is 
1, 2, 5, 6, 7, 9, 12, 15, 18, 19, 27
As mentioned above Q2 is the median of the data. 
Hence Q2 = 9
Q1 is the median of lower half.
So Q1 = 5.5
Q3 is the median of upper half. 
So Q3 = 16.5
Therefore IQR for the given data = Q3 - Q1 = 16.5 - 5.5 = 11

Example:
Data: 12, 9, 6, 5, 2, 26
The data set after being sorted is
2, 5, 6, 9, 12, 26
Q2 is the median of the complete data set Hence Q2 = (6 + 9)/2 = 7.5
But since the number is values is even, Q1 is based off of the lower central value of 6 and the
Q3 is based off of the upper central value of 9.
So Q1 = 5
So Q3 = 12
Therefore the IQR for the given data = Q3 - Q1 = 12 - 5 = 7 
*/

 import java.util.ArrayList;
 import java.util.Arrays;
 import java.util.stream.Collectors;
 
/**
 * Class Usage:
 * 
 * <p>Compute the Five Number Summary (minimum (Q0), Q1, median (Q2), Q3, maximum (Q4)):
 * <p>             IQRfilter.fiveNumberSummary(data)
 * 
 * <p>Find outliers in data:
 * <p>             IQRfilter.findOutliers(data, outlierRangeMultiplier)
 * 
 * <p>Remove outliers in time series data:
 * <p>             IQRfilter smoother = new IQRfilter(windowSize, outlierRangeMultiplier);
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
 * <p>This class must be instantiated only for the running window outlier/smoother function.
 * <Methods for FNS and IQR calculations are static thus no instantiation needed.
 */

public class IQRfilter
{
    private int n; // window size - number of elements in the running window
    private double[] y; // window data storage
    private double threshold; // IQR multiplier to set outlier threshold

    /**
     * 
     * @param WindowSize 4 is a good min; more like 9 would be slow to accept a step change and
     *                   maybe quicker to find a spike
     * @param outlierRangeMultiplier 1.5 is typical; 3.0 rejects only extreme outliers
     */
    public IQRfilter(int WindowSize, double outlierRangeMultiplier)
    {
        this.n = WindowSize;
        this.threshold = outlierRangeMultiplier;
        this.y = new double[WindowSize]; // y here will be the sensor data history [0] the oldest
        for (int i = 0; i < this.y.length; i++)
        {
            this.y[i] = Double.NaN;
        }
    }

    /**
     * An outlier is any data point typically more than 1.5 interquartile ranges (IQRs)
     * below the first quartile or above the third quartile. The IQR multiplier is specified in
     * the class constructor.
     * 
     * <p>This method determines if the most recent point is a data outlier based on IQR of the
     * current window of data and returns either the original value if not an outlier or the 3rd
     * or 1st quartile if an outlier. "Rejected" data is still retained for use in the next window.
     * 
     * <p>Thus it's a sort of spike remover or smoothing method.
     * 
     * <p>This is a little different than the static method that returns a set of outliers
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

            double iqr = getIQR(fns);
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
     * q1 value (inclusive method - median value used if odd number of values)
     * @param values
     * @return q1 value
     */
    static double getQ1(double[] values)
    {
        int sz = values.length;

        if (sz == 1)
            return values[0];
        else if (sz % 2 == 0)
            return getMedian(values, 0, sz / 2 - 1);
        else
            return getMedian(values, 0, sz / 2);
    }

    /**
     * Median value
     * @param values
     * @param from
     * @param to
     * @return median value - center value if odd number of values or average of two central values if even number of values
     */
    static double getMedian(double[] values, int from, int to)
    {
        int sz = to - from + 1;

        if (sz % 2 == 0)
        {
            return (values[from + sz/2] + values[from + (sz/2) - 1] ) / 2.;
        }
        else
            return values[from + (sz / 2)];
    }

     /**
      * Average getter
      * @param values
      * @return average of values
      */
      static double getAverage(double[] values)
      {
        double tot = 0;

        for (double d : values)
        {
            tot += d;            
        }
        return tot / values.length;
    }

    /**
     * q3 value (inclusive method - median value used if odd number of values)
     * @param values
     * @return q3 value
     */
    static double getQ3(double[] values)
    {
        int sz = values.length;

        if (sz == 1)
        {
            return values[0];            
        }
        // same calculation for odd and even numbers because using the inclusive method
        return getMedian(values, sz / 2, sz - 1);
    }

    /**
     * Five Number Summary of the data
     * 
     * <p>CAUTION - input data modified (sorted ascending) upon return
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
     * <p>CAUTION - input data modified (sorted ascending) upon return
     *
     * @param values - input data but MODIFIED upon return in ascending sorted order
     * @param outlierRangeMultiplier - criterion multiplies IQR;
     *          typical values are 1.5=>outlier 3.0=>extreme outlier
     * @return array of outliers of the input values based on input param outlierRangeMultiplier
     */
    static double[] findOutliers(double[] values, double outlierRangeMultiplier)
    {
        ArrayList<Double> outlierList = new ArrayList<Double>();
        double[] fns = fiveNumberSummary(values);
        double iqr = getIQR(fns);
        double outlierBoundary = outlierRangeMultiplier * iqr;
        double lb = fns[1] - outlierBoundary;
        double ub = fns[3] + outlierBoundary;

        for(var value:values)
        {
            if (value < lb || value > ub)
            {
                outlierList.add(value);                
            }
        }
        double[] outliers = outlierList.stream().mapToDouble(i -> i).toArray();
        return outliers;
    }

    /**
     * Interquartile range getter
     * @param fns Five Number Summary array
     * @return IQR of FNS, q3-q1 (a positive number or 0)
     */
    static double getIQR(double[] fns)
    {
        return fns[3] - fns[1];
    }

    /**
     * Format for display a Five Number Summary with Interquartile Range
     * @param fns Five Number Summary array
     * @return formatted string
     */
    static String dumpFnsIqr(double[] fns)
    {
        return String.format("Five Number Summary:[min(q0)=%f, q1=%f, median(q2)=%f, q3=%f, max(q4)=%f]%nInterquartile Range=%f",
            fns[0], fns[1], fns[2], fns[3], fns[4], getIQR(fns));
    }

    /**
     * Format for display a double array
     * @param array
     * @return formatted string
     */
    static String dumpData(double[] array)
    {
        return Arrays.stream(array)
            .mapToObj(i -> String.format("%f", i))
            .collect(Collectors.joining(", ", "[", "]"));
    }

    /**
    * Test main - example usage of five number summary of data and IQR based outliers
    * @param args - not used
    */
    public static void main(String[] args)
    {
        double[][] tests = {
          { -80., -90., -100., -110., -120., -130., -300.},
          { -100., -110., -120., -130., -200., -30.2, -2.},
          { 1., 19., 7., 6., 5., 9., 12., 27., 18., 2., 15. },
          { 1., 200.0, 5.0, 6.0, 9.0, 12.0, -200.0 },
          { 12.0, 9.0, 6.0, 5.0, 2.0, 26.0 },
          { 1., 3., 4., 5., 5., 6., 7., 11. },
          { 2. },
          { 20., 25. },
          { 20., 25., 42.},
          { 90., 87., 80., 65.}
        };
    

        // run all tests - batches of data; data are sorted for IQR processing
        for (double[] test : tests)
        {
            var data = Arrays.copyOf(test, test.length); // protect test data from the IQR sort so they can be used again in other tests

            // Input test data
            System.out.println("\n\nInput:" + IQRfilter.dumpData(data));

            // Sorted data and FiveNumberSummary of the data and IQR
            var fns = IQRfilter.fiveNumberSummary(data);
            System.out.println(IQRfilter.dumpFnsIqr(fns));

            // IQR outliers
            var outlierRangeMultiplier = 1.5;
            var outliers = IQRfilter.findOutliers(data, outlierRangeMultiplier);
            System.out.println("IQR Outliers with range multiplier of " + outlierRangeMultiplier + IQRfilter.dumpData(outliers));
        }

        // run a test simulating a series of real-time stream of data
        int windowSize = 6;
        double threshold = 1.5;
        System.out.println("\n\nfilter data stream with window size " + windowSize + ", outlier range multiplier " + threshold);
        IQRfilter smoother = new IQRfilter(windowSize, threshold);
        for (double[] test : tests)
        {
            for (double element : test)
            {
                System.out.println(element + " -> " + smoother.calculate(element));
            }            
        }
    }
}
/*

Input:[-80.000000, -90.000000, -100.000000, -110.000000, -120.000000, -130.000000, -300.000000]
Five Number Summary:[min(q0)=-300.000000, q1=-125.000000, median(q2)=-110.000000, q3=-95.000000, max(q4)=-80.000000]
Interquartile Range=30.000000
IQR Outliers with range multiplier of 1.5[-300.000000]


Input:[-100.000000, -110.000000, -120.000000, -130.000000, -200.000000, -30.200000, -2.000000]
Five Number Summary:[min(q0)=-200.000000, q1=-125.000000, median(q2)=-110.000000, q3=-65.100000, max(q4)=-2.000000]
Interquartile Range=59.900000
IQR Outliers with range multiplier of 1.5[]


Input:[1.000000, 19.000000, 7.000000, 6.000000, 5.000000, 9.000000, 12.000000, 27.000000, 18.000000, 2.000000, 15.000000]
Five Number Summary:[min(q0)=1.000000, q1=5.500000, median(q2)=9.000000, q3=16.500000, max(q4)=27.000000]
Interquartile Range=11.000000
IQR Outliers with range multiplier of 1.5[]


Input:[1.000000, 200.000000, 5.000000, 6.000000, 9.000000, 12.000000, -200.000000]
Five Number Summary:[min(q0)=-200.000000, q1=3.000000, median(q2)=6.000000, q3=10.500000, max(q4)=200.000000]
Interquartile Range=7.500000
IQR Outliers with range multiplier of 1.5[-200.000000, 200.000000]


Input:[12.000000, 9.000000, 6.000000, 5.000000, 2.000000, 26.000000]
Five Number Summary:[min(q0)=2.000000, q1=5.000000, median(q2)=7.500000, q3=12.000000, max(q4)=26.000000]
Interquartile Range=7.000000
IQR Outliers with range multiplier of 1.5[26.000000]


Input:[1.000000, 3.000000, 4.000000, 5.000000, 5.000000, 6.000000, 7.000000, 11.000000]
Five Number Summary:[min(q0)=1.000000, q1=3.500000, median(q2)=5.000000, q3=6.500000, max(q4)=11.000000]
Interquartile Range=3.000000
IQR Outliers with range multiplier of 1.5[]


Input:[2.000000]
Five Number Summary:[min(q0)=2.000000, q1=2.000000, median(q2)=2.000000, q3=2.000000, max(q4)=2.000000]
Interquartile Range=0.000000
IQR Outliers with range multiplier of 1.5[]


Input:[20.000000, 25.000000]
Five Number Summary:[min(q0)=20.000000, q1=20.000000, median(q2)=22.500000, q3=25.000000, max(q4)=25.000000]
Interquartile Range=5.000000
IQR Outliers with range multiplier of 1.5[]


Input:[20.000000, 25.000000, 42.000000]
Five Number Summary:[min(q0)=20.000000, q1=22.500000, median(q2)=25.000000, q3=33.500000, max(q4)=42.000000]
Interquartile Range=11.000000
IQR Outliers with range multiplier of 1.5[]


Input:[90.000000, 87.000000, 80.000000, 65.000000]
Five Number Summary:[min(q0)=65.000000, q1=72.500000, median(q2)=83.500000, q3=88.500000, max(q4)=90.000000]
Interquartile Range=16.000000
IQR Outliers with range multiplier of 1.5[]


filter data stream with window size 6, outlier range multiplier 1.5
-80.0 -> -80.0
-90.0 -> -90.0
-100.0 -> -100.0
-110.0 -> -110.0
-120.0 -> -120.0
-130.0 -> -130.0
-300.0 -> -130.0
-100.0 -> -100.0
-110.0 -> -110.0
-120.0 -> -120.0
-130.0 -> -130.0
-200.0 -> -200.0
-30.2 -> -100.0
-2.0 -> -2.0
1.0 -> 1.0
19.0 -> 19.0
7.0 -> 7.0
6.0 -> 6.0
5.0 -> 5.0
9.0 -> 9.0
12.0 -> 12.0
27.0 -> 12.0
18.0 -> 18.0
2.0 -> 2.0
15.0 -> 15.0
1.0 -> 1.0
200.0 -> 27.0
5.0 -> 5.0
6.0 -> 6.0
9.0 -> 9.0
12.0 -> 12.0
-200.0 -> 5.0
12.0 -> 12.0
9.0 -> 9.0
6.0 -> 6.0
5.0 -> 5.0
2.0 -> 2.0
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
90.0 -> 42.0
87.0 -> 87.0
80.0 -> 80.0
65.0 -> 65.0
 */