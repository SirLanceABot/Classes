package app;

import java.io.*;
import java.net.*;
import java.util.Arrays;

import com.fasterxml.jackson.databind.ObjectMapper;
import com.fasterxml.jackson.databind.ObjectReader;
import com.fasterxml.jackson.databind.ObjectWriter;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.type.ArrayType;
import com.fasterxml.jackson.databind.DeserializationFeature;

public class App {
    
       public static String getHTML(String urlToRead) throws IOException {
          StringBuilder result = new StringBuilder();
          URL url = new URL(urlToRead);
          HttpURLConnection conn = (HttpURLConnection) url.openConnection();
          conn.setRequestMethod("GET"); // implied so not needed but good to include
          BufferedReader rd = new BufferedReader(new InputStreamReader(conn.getInputStream()));
          String line;
          while ((line = rd.readLine()) != null) {
             result.append(line);
          }
          rd.close();
          return result.toString();
       }

       public static void main(String[] args) throws Exception {

        // a example of reading an unnamed json int.
        {
        String json = "123"; // cannot have {} or []
        int anInt = new ObjectMapper().readValue(json, int.class);
        System.out.println("single int = " + anInt);
        }

        // a example of reading an unnamed json array of int. The [] surround array elements.
        {
        String json = "[1, 2, 3]";
        int[] array = new ObjectMapper().readValue(json, int[].class);
        System.out.println("array of ints = " + Arrays.toString(array));
        }
  
        try{

         // read the http json string from the URL  
         //System.out.println(getHTML(args[0])); // use if URL passed in as an arg
         String Lines_GRIP_HTTP = getHTML("http://127.0.0.1:2084/GRIP/data"); // get data from this machine running GRIP
         System.out.println(Lines_GRIP_HTTP); // string as read
  
         // ignore unknown properties for all classes
         // for a single class use the @JsonIgnoreProperties(ignoreUnknown = true) and its import in front of the class
         //   
         // mapper.configure(DeserializationFeature.FAIL_ON_UNKNOWN_PROPERTIES, false);

         ObjectMapper mapper = new ObjectMapper(); // Jackson json to/from java object mapper. Use for all processes since it's the same class
  
         // parse json into java object - one way of doing it
         String json3 = Lines_GRIP_HTTP;
         //String json3 = "{\"myLinesReport\":[{\"length\":[14,13],\"y1\":[11],\"x1\":[22],\"x2\":[33],\"y2\":[44]}]}";
         //String json3 = "{\"length\":14,\"y1\":11,\"x1\":22,\"x2\":33,\"y2\":44}";
         MyLinesReport class3 = mapper.readValue(json3, MyLinesReport.class); // one way to read the json string and convert to java object
         System.out.println("class3: " + class3); // display the java object

         // serialize the java object into a json text String - use the java object created from the above parse process
       // convert java object to json string
       ObjectWriter WRITER = mapper.writerFor(MyLinesReport.class);
       String myJSON = WRITER.writeValueAsString(class3);
       //System.out.println("myJSON= " + myJSON); // line returns; doesn't display well if at all
       System.out.println("myJSON=");
       for (int i=0; i<myJSON.length(); i+=80){
         System.out.println(myJSON.substring(i, Math.min(i+80, myJSON.length()))); // print some on each line
       }

      // parse json into java object - another way of doing it
      // use the json string created from the above serialization - should match the original string
       ObjectReader READER = mapper.readerFor(MyLinesReport.class);
       MyLinesReport class4 = null;
       class4 = READER.readValue(myJSON); // another way to read the json string and convert to java object

       System.out.println("class4: " + class4); // display the java object
      }
      catch(Exception e){e.printStackTrace();}

// { "myLinesReport": { "length": [ ,  ,  ]  ,    "y1": [ ,, ],    "x1": [,, ] ,    "x2": [  ,,] ,   "y2": [,, ], "angle": [ , ,] }}

    }
}

// something from WPILIB that uses Jackson json to use as a teaching model
// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2019-2020 FIRST. All Rights Reserved.                        */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package edu.wpi.first.wpilibj.trajectory;

// import java.io.BufferedReader;
// import java.io.BufferedWriter;
// import java.io.IOException;
// import java.nio.file.Files;
// import java.nio.file.Path;
// import java.util.Arrays;

// import com.fasterxml.jackson.core.JsonProcessingException;
// import com.fasterxml.jackson.databind.ObjectMapper;
// import com.fasterxml.jackson.databind.ObjectReader;
// import com.fasterxml.jackson.databind.ObjectWriter;

// public final class TrajectoryUtil {
//   private static final ObjectReader READER = new ObjectMapper().readerFor(Trajectory.State[].class);
//   private static final ObjectWriter WRITER = new ObjectMapper().writerFor(Trajectory.State[].class);

//   private TrajectoryUtil() {
//     throw new UnsupportedOperationException("This is a utility class!");
//   }

//   /**
//    * Imports a Trajectory from a PathWeaver-style JSON file.
//    * @param path the path of the json file to import from
//    * @return The trajectory represented by the file.
//    * @throws IOException if reading from the file fails
//    */
//   public static Trajectory fromPathweaverJson(Path path) throws IOException {
//     try (BufferedReader reader = Files.newBufferedReader(path)) {
//       Trajectory.State[] state = READER.readValue(reader);
//       return new Trajectory(Arrays.asList(state));
//     }
//   }

//   /**
//    * Exports a Trajectory to a PathWeaver-style JSON file.
//    * @param trajectory the trajectory to export
//    * @param path the path of the file to export to
//    * @throws IOException if writing to the file fails
//    */
//   public static void toPathweaverJson(Trajectory trajectory, Path path) throws IOException {
//     Files.createDirectories(path.getParent());
//     try (BufferedWriter writer = Files.newBufferedWriter(path)) {
//       WRITER.writeValue(writer, trajectory.getStates().toArray(new Trajectory.State[0]));
//     }
//   }

//   /**
//    * Deserializes a Trajectory from PathWeaver-style JSON.
//    * @param json the string containing the serialized JSON
//    * @return the trajectory represented by the JSON
//    * @throws JsonProcessingException if deserializing the JSON fails
//    */
//   public static Trajectory deserializeTrajectory(String json) throws JsonProcessingException {
//     Trajectory.State[] state = READER.readValue(json);
//     return new Trajectory(Arrays.asList(state));
//   }

//   /**
//    * Serializes a Trajectory to PathWeaver-style JSON.
//    * @param trajectory the trajectory to export
//    * @return the string containing the serialized JSON
//    * @throws JsonProcessingException if serializing the Trajectory fails
//    */
//   public static String serializeTrajectory(Trajectory trajectory) throws JsonProcessingException {
//     return WRITER.writeValueAsString(trajectory.getStates().toArray(new Trajectory.State[0]));
//   }
// }


// compile ‘com.googlecode.json-simple:json-simple:1.1.1’ inside dependies {} in build.gradle

// <classpathentry kind\="lib" path\="C\:/Users/Public/wpilib/2020/maven/com/fasterxml/jackson/core/jackson-databind/2.10.0/jackson-databind-2.10.0.jar" sourcepath\="C\:/Users/Public/wpilib/2020/maven/com/fasterxml/jackson/core/jackson-databind/2.10.0/jackson-databind-2.10.0-sources.jar">
// 	<attributes>
// 		<attribute name\="gradle_used_by_scope" value\="main,test"/>
// 	</attributes>
// </classpathentry>
// <classpathentry kind\="lib" path\="C\:/Users/Public/wpilib/2020/maven/com/fasterxml/jackson/core/jackson-annotations/2.10.0/jackson-annotations-2.10.0.jar" sourcepath\="C\:/Users/Public/wpilib/2020/maven/com/fasterxml/jackson/core/jackson-annotations/2.10.0/jackson-annotations-2.10.0-sources.jar">
// 	<attributes>
// 		<attribute name\="gradle_used_by_scope" value\="main,test"/>
// 	</attributes>
// </classpathentry>
// <classpathentry kind\="lib" path\="C\:/Users/Public/wpilib/2020/maven/com/fasterxml/jackson/core/jackson-core/2.10.0/jackson-core-2.10.0.jar" sourcepath\="C\:/Users/Public/wpilib/2020/maven/com/fasterxml/jackson/core/jackson-core/2.10.0/jackson-core-2.10.0-sources.jar">
// 	<attributes>
// 		<attribute name\="gradle_used_by_scope" value\="main,test"/>
// 	</attributes>
// </classpathentry>

/*

 a contours report from early testing
{  "myContoursReport": {    "area": [      14534.0,      12318.0    ],    "centerY": [      323.0,      91.0    ],    "centerX": [      303.0,      369.0   
 ],    "width": [      129.0,      247.0    ],    "solidity": [      0.5692352883579751,      0.4085097915664848    ],    "height": [      282.0,      181.0    ]  }}
==============================================
GRIP Lines report testing

single int = 123
array of ints = [1, 2, 3]
{  "myLinesReport": {    "length": [      13.975412810455344,      13.974741058617209,      10.057269375680345,      11.849181090557229,      10.302046490932394,      186.28720621958493,      131.25106356465184,      18.19988239464849,      9.010813632120417,      17.406320566760336,      12.562121623766508,   
   29.044470591409553,      16.984730868599964,      7.010681384637,      15.811376897673428,      3.7367868696757904,      27.299904930180467,      28.43878962769642,      12.31078520231436,      16.43478381813125,      7.282438451987197,      15.950860531051514,      9.518797173497838    ],    "y1": [      117.95355987548828,      156.85401916503906,      178.30584716796875,      189.0358123779297,      181.79782104492188,      184.34530639648438,      191.3836669921875,      406.92718505859375,      399.48638916015625,      407.2923889160156,      420.6014709472656,      447.03485107421875,      427.71893310546875,      432.2363586425781,      436.13336181640625,      441.524169921875,      443.31036376953125,      469.21014404296875,      459.26513671875,      460.68304443359375,      463.609619140625,      467.4995422363281,      476.8946838378906    ],    "angle": [      116.50214605149823,      99.73638371931891,   
   100.784911886077,      -73.84102221577156,      15.94404370615603,      1.0158556267107977,      1.040266602119558,      -74.42435911715773,      147.79986889688178,      160.21484541669926,      -95.38184830650987,      -118.05675656827916,      165.4683536423023,      142.49287365530333,      161.4796879261021,      175.18830928867854,      164.40497390017782,      -131.76233733927756,      66.42607131663314,      100.34247267151466,      151.407571992754,   
   136.50659666693454,      -156.0082474275815    ],    "y2": [      130.4604034423828,      170.62747192382812,      188.1854705810547,      177.65475463867188,      184.62777709960938,      187.64801025390625,      193.76654052734375,      389.3956604003906,      404.2880554199219,      413.184326171875,     
 408.0947265625,      421.40362548828125,      431.98065185546875,      436.5048828125,      441.15570068359375,      441.8376159667969,      450.6495666503906,      447.99725341796875,      470.54852294921875,      476.8507995605469,      467.0948181152344,      478.4780578613281,      473.0242919921875    ], 
   "x1": [      244.0311737060547,      253.00271606445312,      256.5743713378906,      256.95440673828125,      490.6470642089844,      44.375526428222656,      504.3839111328125,      253.3122100830078,      251.94512939453125,      242.025146484375,      254.62466430664062,      267.8250732421875,      184.26974487304688,      167.15234375,      160.79530334472656,      145.59547424316406,      141.92674255371094,      287.0596008300781,      253.37681579589844,      238.44302368164062,      87.1391372680664,      80.03158569335938,      303.1162414550781    ],    "x2": [      237.79490661621094,      250.63937377929688,      254.6924285888672,      260.2520751953125,      500.55279541015625,      230.63345336914062,      635.6133422851562,      258.1990661621094,  
    244.32025146484375,      225.64634704589844,      253.44642639160156,      254.16412353515625,      167.828369140625,      161.59092712402344,      145.8027801513672,      141.87185668945312,      115.6318588256836,      268.1181640625,      258.30029296875,      235.49246215820312,      80.74481964111328, 
     68.45997619628906,      294.4198303222656    ]  }}
class3: length:14.0 (244, 118)-(238, 130)
length:14.0 (253, 157)-(251, 171)
length:10.1 (257, 178)-(255, 188)
length:11.8 (257, 189)-(260, 178)
length:10.3 (491, 182)-(501, 185)
length:186.3 (44, 184)-(231, 188)
length:131.3 (504, 191)-(636, 194)
length:18.2 (253, 407)-(258, 389)
length:9.0 (252, 399)-(244, 404)
length:17.4 (242, 407)-(226, 413)
length:12.6 (255, 421)-(253, 408)
length:29.0 (268, 447)-(254, 421)
length:17.0 (184, 428)-(168, 432)
length:7.0 (167, 432)-(162, 437)
length:15.8 (161, 436)-(146, 441)
length:3.7 (146, 442)-(142, 442)
length:27.3 (142, 443)-(116, 451)
length:28.4 (287, 469)-(268, 448)
length:12.3 (253, 459)-(258, 471)
length:16.4 (238, 461)-(235, 477)
length:7.3 (87, 464)-(81, 467)
length:16.0 (80, 467)-(68, 478)
length:9.5 (303, 477)-(294, 473)

myJSON=
{"myLinesReport":{"length":[13.975412810455344,13.974741058617209,10.05726937568
0345,11.849181090557229,10.302046490932394,186.28720621958493,131.25106356465184
,18.19988239464849,9.010813632120417,17.406320566760336,12.562121623766508,29.04
4470591409553,16.984730868599964,7.010681384637,15.811376897673428,3.73678686967
57904,27.299904930180467,28.43878962769642,12.31078520231436,16.43478381813125,7
.282438451987197,15.950860531051514,9.518797173497838],"y1":[117.95355987548828,
156.85401916503906,178.30584716796875,189.0358123779297,181.79782104492188,184.3
4530639648438,191.3836669921875,406.92718505859375,399.48638916015625,407.292388
9160156,420.6014709472656,447.03485107421875,427.71893310546875,432.236358642578
1,436.13336181640625,441.524169921875,443.31036376953125,469.21014404296875,459.
26513671875,460.68304443359375,463.609619140625,467.4995422363281,476.8946838378
906],"x1":[244.0311737060547,253.00271606445312,256.5743713378906,256.9544067382
8125,490.6470642089844,44.375526428222656,504.3839111328125,253.3122100830078,25
1.94512939453125,242.025146484375,254.62466430664062,267.8250732421875,184.26974
487304688,167.15234375,160.79530334472656,145.59547424316406,141.92674255371094,
287.0596008300781,253.37681579589844,238.44302368164062,87.1391372680664,80.0315
8569335938,303.1162414550781],"y2":[130.4604034423828,170.62747192382812,188.185
4705810547,177.65475463867188,184.62777709960938,187.64801025390625,193.76654052
734375,389.3956604003906,404.2880554199219,413.184326171875,408.0947265625,421.4
0362548828125,431.98065185546875,436.5048828125,441.15570068359375,441.837615966
7969,450.6495666503906,447.99725341796875,470.54852294921875,476.8507995605469,4
67.0948181152344,478.4780578613281,473.0242919921875],"x2":[237.79490661621094,2
50.63937377929688,254.6924285888672,260.2520751953125,500.55279541015625,230.633
45336914062,635.6133422851562,258.1990661621094,244.32025146484375,225.646347045
89844,253.44642639160156,254.16412353515625,167.828369140625,161.59092712402344,
145.8027801513672,141.87185668945312,115.6318588256836,268.1181640625,258.300292
96875,235.49246215820312,80.74481964111328,68.45997619628906,294.4198303222656]}
}
class4: length:14.0 (244, 118)-(238, 130)
length:14.0 (253, 157)-(251, 171)
length:10.1 (257, 178)-(255, 188)
length:11.8 (257, 189)-(260, 178)
length:10.3 (491, 182)-(501, 185)
length:186.3 (44, 184)-(231, 188)
length:131.3 (504, 191)-(636, 194)
length:18.2 (253, 407)-(258, 389)
length:9.0 (252, 399)-(244, 404)
length:17.4 (242, 407)-(226, 413)
length:12.6 (255, 421)-(253, 408)
length:29.0 (268, 447)-(254, 421)
length:17.0 (184, 428)-(168, 432)
length:7.0 (167, 432)-(162, 437)
length:15.8 (161, 436)-(146, 441)
length:3.7 (146, 442)-(142, 442)
length:27.3 (142, 443)-(116, 451)
length:28.4 (287, 469)-(268, 448)
length:12.3 (253, 459)-(258, 471)
length:16.4 (238, 461)-(235, 477)
length:7.3 (87, 464)-(81, 467)
length:16.0 (80, 467)-(68, 478)
length:9.5 (303, 477)-(294, 473)

*/