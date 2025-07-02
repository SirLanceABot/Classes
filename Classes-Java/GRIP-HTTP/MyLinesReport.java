package app;

// json ata from GRIP lines report
// { "myLinesReport": { "length": [ ,  ,  ]  ,    "y1": [ ,, ],    "x1": [,, ] ,    "x2": [  ,,] ,   "y2": [,, ], "angle": [ , ,] }}

public class MyLinesReport{

    public Data myLinesReport;
    MyLinesReport(){}
    public String toString(){
        return myLinesReport.toString();
    }
  }

