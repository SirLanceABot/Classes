package app;

import com.fasterxml.jackson.annotation.JsonIgnoreProperties;

@JsonIgnoreProperties(ignoreUnknown = true)

    public class Data{
        public double[] length;
        public double[] y1, x1, y2, x2;
        //public double[] angle;
        // the input data include field "angle" however for example it is ignored
        // by the import and annotation included above
        Data (){}

 
        public String toString() {

            String returnIt = "";
            for (int i=0; i<length.length; i++) {
                returnIt += String.format(
                    "length:%.1f (%.0f, %.0f)-(%.0f, %.0f)\n",
                    length[i], x1[i], y1[i], x2[i], y2[i]);
            }
            return returnIt;
        }
    }