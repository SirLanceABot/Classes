/*
Five Number Summary of data and IQR based outliers


Quantiles are the set of values/points that divides the dataset into groups of equal size.

if 4 parts then they are quartiles
In descriptive statistics, the interquartile range (IQR) tells you the spread of the middle half of your distribution.

The quartiles of a ranked set of data values are three points which divide the data into exactly four equal parts, each part comprising of quarter data.

Q1 is defined as the middle number between the smallest number and the median of the data set.
Q2 is the median of the data.
Q3 is the middle value between the median and the highest value of the data set.

The interquartile range IQR tells us the range 
where the bulk of the values lie. The interquartile 
range is calculated by subtracting the first quartile
from the third quartile. 
IQR = Q3 - Q1

https://levelup.gitconnected.com/how-to-easily-forecast-the-stock-price-probabilities-time-series-using-iqr-interquartile-range-9d97dafd5a32
https://en.wikipedia.org/wiki/Interquartile_range

Uses 
1. Unlike range, IQR tells where the majority of data lies and is thus preferred over range. 
2. IQR can be used to identify outliers in a data set. 
3. Gives the central tendency of the data. 
Examples: 

Input : 1, 19, 7, 6, 5, 9, 12, 27, 18, 2, 15
Output : 13
The data set after being sorted is 
1, 2, 5, 6, 7, 9, 12, 15, 18, 19, 27
As mentioned above Q2 is the median of the data. 
Hence Q2 = 9
Q1 is the median of lower half, taking Q2 as pivot.
So Q1 = 5
Q3 is the median of upper half talking Q2 as pivot. 
So Q3 = 18
Therefore IQR for given data=Q3-Q1=18-5=13 

Input : 1, 3, 4, 5, 5, 6, 7, 11
Output : 3

*/

// based on github jbosstm  narayana

/*
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

import java.util.*;

public class IQR {
    /**
     * Test main - example usage of five number summary of data and IQR based outliers
     * @param args - not used
     */
    public static void main(String[] args) {
        double[][] tests = {
           { 1., 200.0, 5.0, 6.0, 9.0, 12.0 },
           { 2.0, 5.0, 6.0, 9.0, 12.0, 26.0 },
           { 1, 19, 7, 6, 5, 9, 12, 27, 18, 2, 15 },
           { 1, 3, 4, 5, 5, 6, 7, 11 },
           { 2.0 },
           { 20., 25. },
           { 20., 25., 42.},
           { 90., 87., 80., 65.}
        };

        for (double[] data : tests) { // run all tests
            
            // FiveNumberSummary of the data and IQR
            double[] fns = fiveNumberSummary(data);
            System.out.printf("%nmin(q0)=%f %nq1=%f %nmedian(q2)=%f %nq3=%f %nmax(q4)=%f %nIQR=%f%n",
                fns[0], fns[1], fns[2], fns[3], fns[4], fns[3]-fns[1]);

            // IQR outliers
            System.out.printf("Outliers:%n");
            for(var outlier: findOutliers(data, 1.5))
                System.out.println(outlier);
        }
    }

    /**
     * FiveNumberSummary of the data and IQR can be calculated from q3-q1
     * @param values - input data but returned in ascending sorted order
     * @return min(q0), q1, median(q2), q3, max(q4)
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
     * @param values - input data but returned in ascending sorted order
     * @param outlierRange - criterion multiples IQR
     *          typical values are 1.5=>outlier 3.0=>extreme outlier
     * @return list of outliers based on input param outlierRange
     */
    static ArrayList<Double> findOutliers(double[] values, double outlierRange) {
        ArrayList<Double> outliers = new ArrayList<Double>();
        double[] fns = fiveNumberSummary(values);
        double iqr = fns[3] - fns[1]; // the interquartile range
        double outlierBoundary = outlierRange * iqr;
        double lb = fns[1] - outlierBoundary;
        double ub = fns[3] + outlierBoundary;

        for(var value:values)
        {
            if (value < lb || value > ub)
                outliers.add(value);
        }
        return outliers;
    }

    static double getMedian(double[] values, int from, int to) {
        int sz = to - from + 1;

        if (sz % 2 == 0)
             return (values[from + sz/2] + values[from + (sz/2) - 1] ) / 2;
        else
            return values[from + (sz / 2)];
    }

    static double getQ1(double[] values) {
        int sz = values.length;

        if (sz == 1)
            return values[0];
        else if (sz % 2 == 0)
            return getMedian(values, 0, sz / 2 - 1);
        else
            return getMedian(values, 0, sz / 2 - 1);
    }

    static double getQ3(double[] values) {
        int sz = values.length;

        if (sz == 1)
            return values[0];
        else if (sz % 2 == 0)
            return getMedian(values, sz / 2, sz - 1);
        else
            return getMedian(values, sz / 2 + 1, sz - 1);
    }

    static double getAverage(double[] values) {
        double tot = 0;

        for (double d : values)
            tot += d;

        return tot / values.length;
    }

    static String dump(double[] fns) {
        return String.format("min(q0)=%f q1=%f median(q2)=%f q3=%f max(q4)=%f IQR=%f",
                fns[0], fns[1], fns[2], fns[3], fns[4], fns[3]-fns[1]);
    }
}
/* output

min(q0)=1.000000    
q1=5.000000
median(q2)=7.500000 
q3=12.000000        
max(q4)=200.000000
IQR=7.000000
Outliers:
200.0

min(q0)=2.000000
q1=5.000000
median(q2)=7.500000
q3=12.000000
max(q4)=26.000000
IQR=7.000000
Outliers:
26.0

min(q0)=1.000000
q1=5.000000
median(q2)=9.000000
q3=18.000000
max(q4)=27.000000 
IQR=13.000000
Outliers:

min(q0)=1.000000
q1=3.500000
median(q2)=5.000000
q3=6.500000
max(q4)=11.000000
IQR=3.000000
Outliers:

min(q0)=2.000000
q1=2.000000
median(q2)=2.000000
q3=2.000000
max(q4)=2.000000
IQR=0.000000
Outliers:

min(q0)=20.000000 
q1=20.000000 
median(q2)=22.500000
q3=25.000000
max(q4)=25.000000
IQR=5.000000
Outliers:

min(q0)=20.000000
q1=20.000000
median(q2)=25.000000
q3=42.000000
max(q4)=42.000000
IQR=22.000000
Outliers:

min(q0)=65.000000
q1=72.500000
median(q2)=83.500000
q3=88.500000
max(q4)=90.000000 
IQR=16.000000
Outliers:

*/