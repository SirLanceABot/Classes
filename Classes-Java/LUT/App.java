// Example of LUT - Table Lookup

package app;

public class App {

    public static void main(String[] args)
    {
        App x = new App(); // create non-static object

        LUT conversion = x.new LUT(10); // construct with parameter at least as large as the number of data points; minimum of 2 points

        conversion.add(10., 100.); // enter X, Y co-ordinate data in X order ascending
        conversion.add(2., 200.); // must add at least 2 data points

        System.out.println("Converted to " + conversion.lookup(1.5)); // lookup returns the value of Y coresponding to the X parameter
        System.out.println("Converted to " + conversion.lookup(1.0)); // lookup returns the value of Y coresponding to the X parameter
        System.out.println("Converted to " + conversion.lookup(Double.POSITIVE_INFINITY)); // lookup returns the value of Y coresponding to the X parameter
        System.out.println("Converted to " + conversion.lookup(Double.NEGATIVE_INFINITY)); // lookup returns the value of Y coresponding to the X parameter
        System.out.println("Converted to " + conversion.lookup(-99999999.)); // lookup returns the value of Y coresponding to the X parameter

        System.out.println(conversion); // print the whole table
    }
}
