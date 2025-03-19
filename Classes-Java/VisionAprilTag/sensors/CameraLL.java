package frc.robot.sensors;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import java.util.stream.Collectors;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Alert.AlertType;

/**
 * Implementation of the camera class for Limelights
 * <p>This class acquires the Limelight target data and the MegaTag2 robot pose in field.
 * <p>Those are two mostly separate entities with some getters for the target and some
 * getters for the MegaTag2 pose.
 * <p>It is a highly stripped-down version of the LimelightHelpers with efficient
 * use of NetworkTables Publisher/Subscriber techniques.
 * <p>If additional functionality is needed, contact your mentors to add to this class.
 * <p>Typical usage:
 * 
 <pre>

    // construct the limelight object once
    CameraLL LL1;
    boolean useLL1 = true;

    LL1 = useLL1 ? CameraLL.makeCamera("limelight") : null; // make a limelight - null if it doesn't exist or not requested

    // set the camera stream mode as often as necessary
    if (LL1 != null)
    {
      // example of two ways to specify the view of the external camera (method overloads)
      LL1.setStreamMode_Standard();
      LL1.setStreamMode_PiPMain();
      LL1.setStreamMode_PiPSecondary();
      // OR
      LL1.setStreamMode(streamMode);
    }

    // periodically set and get data
    if (LL1 != null)
    {
      // for MegaTag2 set the robot orientation from the gyro heading and rate.
      //FIXME get the gyro values somehow but here are zeros for test data - limits what AprilTags make sense
      LL1.setRobotOrientation(0., 0., 0., 0., 0., 0.);

      LL1.update();

      // some methods to use:
      if (LL1.isFresh())
      {
          LL1.publishPose3d();

          // --- TARGET  STATISTICS ---
          System.out.println(LL1.getTX() + " tx");
          System.out.println(LL1.getTXNC() + " txnc");
          System.out.println(LL1.getTY() + " ty");
          System.out.println(LL1.getTYNC() + " tync");
          System.out.println(LL1.getTA() + " area");
          System.out.println(LL1.getTID() + " tag id");

          // --- MEGATAG2 POSE ---
          System.out.println(LL1.getTimestampSeconds() + " Time of the pose");
          System.out.println(LL1.getPose2d() + " MegaTag2 pose 2-D");
          System.out.println(LL1.getPose3d() + " MegaTag2 pose 3-D");
          System.out.println(LL1.getLatency() + " total latency of the pose");
          System.out.println(LL1.getTagCount() + " number of tags seen to make the pose");
          System.out.println(LL1.getTagSpan() + " span of the tags");
          System.out.println(LL1.getAvgTagDist() + " average distance from tags to robot");
          System.out.println(LL1.getAvgTagArea() + " average area of the tags in their frames\n");

          System.out.println(LL1); // prints same data as above plus more statistics
      }

      // simplistic data usage example for pose estimation
      // m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
      // m_poseEstimator.addVisionMeasurement(LL1.getPose2d(), LL1.getTimestampSeconds());
      }
    }
</pre>
 */
public class CameraLL extends CameraLance {

    private final String name; // name of the limelight
    private final DoubleArraySubscriber t2d;
    private final DoubleArraySubscriber botpose_orb_wpiblue;
    private final DoubleArrayPublisher robot_orientation_set;
    private final DoublePublisher stream;

    // fields associated with statistics and target    
    private TimestampedDoubleArray stats;
    private double tx;
    private double ty;
    private double txnc;
    private double tync;
    private double ta;
    private int tid;
    private boolean isFresh;
    private long previousTimestamp = 0; // arbitrary initial time to start
     
    // fields associated with MegaTag2 blue pose
    private Pose2d pose2d;
    private double timestampSeconds;
    private double latency;
    private int tagCount;
    private double tagSpan;
    private double avgTagDist;
    private double avgTagArea;
    private Pose3d pose3d;
  
    private CameraLL(String name) {
        super(name);
        if (!isAvailable(name))
        {
          throw new RuntimeException("Limelight named " + name + " is not available");
        }

        this.name = name;

        // Get the limelight table
        var table = CameraLance.NTinstance.getTable(name);
     
        /*
        t2d
        	doubleArray containing several values for matched-timestamp statistics:
                targetValid, [0]
                targetCount, [1]
                targetLatency, [2]
                captureLatency, [3]
                tx, [4]
                ty, [5]
                txnc, [6]
                tync, [7]
                ta, [8]
                tid, [9]
                targetClassIndexDetector, [10]
                targetClassIndexClassifier, [11]
                targetLongSidePixels, [12]
                targetShortSidePixels, [13]
                targetHorizontalExtentPixels, [14]
                targetVerticalExtentPixels, [15]
                targetSkewDegrees [16]
         */
        t2d = table.getDoubleArrayTopic("t2d").subscribe(new double[]{}); // default is no data (array length = 0)

        /*
        botpose_orb_wpiblue
        	doubleArray
                Robot transform in field-space (Megatag2 blue driverstation WPILIB origin).
                Translation (X,Y,Z) in meters, [0-2]
                Rotation(Roll,Pitch,Yaw) in degrees, [3-5]
                total latency (cl+tl), [6]
                tag count, [7]
                tag span, [8]
                average tag distance from camera, [9]
                average tag area (percentage of image) [10]
         */
        botpose_orb_wpiblue = table.getDoubleArrayTopic("botpose_orb_wpiblue").subscribe(new double[]{}); // default is no data (array length = 0)

        /*
        robot_orientation_set
        	doubleArray
                SET Robot Orientation and angular velocities in degrees and degrees per second[yaw, yawrate, pitch, pitchrate, roll, rollrate]
         */
        robot_orientation_set = table.getDoubleArrayTopic("robot_orientation_set").publish();

        /*
        stream
            	Sets limelight's streaming mode
                    0	Standard - Side-by-side streams if a webcam is attached to Limelight
                    1	PiP Main - The secondary camera stream is placed in the lower-right corner of the primary camera stream
                    2	PiP Secondary - The primary camera stream is placed in the lower-right corner of the secondary camera stream
         */
        stream = table.getDoubleTopic("stream").publish();
    }

    /**
     * Essentially the constructor for a limelight object
     * @param name set in the limelight
     * @return a limelight object or null if it doesn't exist
     */
    public static CameraLL makeCamera(String name)
    {
        if (!isAvailable(name))
        {
            return null;
        }

        return new CameraLL(name);
    }

    /**
     * Enables standard viewing of cameras mode. If both internal and external cameras available they are viewed side-by-side.
     */
    public void setStreamMode_Standard()
    {
        stream.set(0.);
    }

    /**
     * Enables Picture-in-Picture mode viewing internal camera with external stream in the corner.
     */
    public void setStreamMode_PiPMain()
    {
        stream.set(1.);
    }

    /**
     * Enables Picture-in-Picture mode viewing external camera with primary camera in the corner.
     */
    public void setStreamMode_PiPSecondary()
    {
        stream.set(2.);
    }

    public enum StreamMode {Both, External, Internal}
    public void setStreamMode(StreamMode streamMode)
    {
        stream.set((double)streamMode.ordinal());
    }

    /**
     * Sets robot orientation values used by MegaTag2 localization algorithm.
     * 
     * @param limelightName Name/identifier of the Limelight
     * @param yaw Robot yaw in degrees. 0 = robot facing red alliance wall in FRC
     * @param yawRate (Unnecessary) Angular velocity of robot yaw in degrees per second
     * @param pitch (Unnecessary) Robot pitch in degrees 
     * @param pitchRate (Unnecessary) Angular velocity of robot pitch in degrees per second
     * @param roll (Unnecessary) Robot roll in degrees
     * @param rollRate (Unnecessary) Angular velocity of robot roll in degrees per second
     */
    public void setRobotOrientation(double yaw, double yawRate, double pitch, double pitchRate, double roll, double rollRate)
    {
        double[] gyro = new double[6];
        gyro[0] = yaw;
        gyro[1] = yawRate;
        gyro[2] = pitch;
        gyro[3] = pitchRate;
        gyro[4] = roll;
        gyro[5] = rollRate;
        robot_orientation_set.set(gyro); // robot orientation sent to LL for MegaTag2
        CameraLance.NTinstance.flush();
    }

    /**
     * getter for validity of last acquisition
     * <p>data must be valid and different timestamp than the previous data
     * 
     * @return freshness of the last acquisition
     */
    public boolean isFresh()
    {
      return isFresh;
    }

              /**
               * 
               * GETTERS FOR TARGET DATA 
               * 
               */


    /**
     * Gets the horizontal offset from the crosshair to the target in degrees.
     * <p>not MegaTag2 pose
     * @return Horizontal offset angle in degrees
     */
    public double getTX()
    {
      return tx;
    }

    /**
     * Gets the vertical offset from the crosshair to the target in degrees.
     * <p>not MegaTag2 pose
     * @return Vertical offset angle in degrees
     */
    public double getTY() {
        return ty;
    }

    /**
     * Gets the horizontal offset from the principal pixel/point to the target in degrees.
     * <p>This is the most accurate 2d metric if you are using a calibrated camera and you don't need adjustable crosshair functionality.
     * <p>not MegaTag2 pose
     * @return Horizontal offset angle in degrees
     */
    public double getTXNC()
    {
        return txnc;
    }

    /**
     * Gets the vertical offset from the principal pixel/point to the target in degrees.
     * <p>This is the most accurate 2d metric if you are using a calibrated camera and you don't need adjustable crosshair functionality.
     * <p>not MegaTag2 pose
     * @return Vertical offset angle in degrees
     */
    public double getTYNC()
    {
        return tync;
    }

    /**
     * Gets the area of the target
     * <p>not MegaTag2 pose
     * @return area of the target
     */
    public double getTA()
    {
        return ta;
    }

    /**
     * Primary targeted tag id for use with getTX, getTXNC, getTY, getTYNC
     * <p>Beware of usage with pose2d or pose3d as a tag id is not an inherent property of the pose
     * <p>not MegaTag2 pose
     * @return tag id
     */
    public int getTID()
    {
        return tid;
    }

            /**
             * 
             * GETTERS FOR MEGATAG2 POSE
             * 
             */
            
    /**
     * MegaTag2 blue pose getter
     * <p>use for PoseEstimator.addVisionMeasurement
     * <p>not Target data
     * @return
     */
    public Pose2d getPose2d()
    {
      return pose2d;
    }

    /**
     * MegaTag2 blue pose getter
     * <p>not Target data
     * @return
     */
    public Pose3d getPose3d()
    {
      return pose3d;
    }

    /**
     * MegaTag2 Timestamp getter from pose
     * <p>use for PoseEstimator.addVisionMeasurement
     * <p>not Target data
      * @return
     */
    public double getTimestampSeconds()
    {
        return timestampSeconds;
    }

    /**
     * MegaTag2 Total latency getter
     * <p>not Target data
     * @return
     */
    public double getLatency()
    {
        return latency;
    }

    /**
     * MegaTag2 Tag count getter
     * <p>not Target data
     * @return
     */
    public int getTagCount()
    {
        return tagCount;
    }

    /**
     * MegaTag2 Tag span getter
     * <p>not Target data
     * @return
     */
    public double getTagSpan()
    {
        return tagSpan;
    }

    /**
     * MegaTag2 Average tag distance getter
     * <p>not Target data
     * @return
     */
    public double getAvgTagDist()
    {
        return avgTagDist;
    }

    /**
     * MegaTag2 Average tag area getter
     * <p>not Target data
     * @return
     */
    public double getAvgTagArea()
    {
        return avgTagArea;
    }

    /**
     * Read the latest Limelight values.
     * <p>Call this method in the robot iterative loop.
     */
    public void update() {

        // get LL data needed to determine validity
        stats = t2d.getAtomic();

        // check if new data
        isFresh = stats.timestamp != previousTimestamp && stats.value.length >= 17 && stats.value[0] == 1.0;
        previousTimestamp = stats.timestamp;

        if (isFresh())
        {
            tx = stats.value[4];
            ty = stats.value[5];
            txnc = stats.value[6];
            tync = stats.value[7];
            ta = stats.value[8];
            tid = (int)stats.value[9];

            var poseRaw = botpose_orb_wpiblue.getAtomic(); // get the LL MegaTag2 pose data

            if (poseRaw.value.length >= 11)
            {
                pose3d = new Pose3d(poseRaw.value[0], poseRaw.value[1], poseRaw.value[2], new Rotation3d(poseRaw.value[3], poseRaw.value[4], poseRaw.value[5])); // robot in field 3d pose
                pose2d = new Pose2d(poseRaw.value[0], poseRaw.value[1], new Rotation2d(Units.degreesToRadians(poseRaw.value[5]))); // robot in field 2d pose
                latency = poseRaw.value[6];
                timestampSeconds = (poseRaw.timestamp / 1000000.0) - (latency / 1000.0); // Convert server timestamp from microseconds to seconds and adjust for latency
                tagCount = (int)poseRaw.value[7];
                tagSpan = poseRaw.value[8];
                avgTagDist = poseRaw.value[9];
                avgTagArea = poseRaw.value[10];
            }
            else
            {
                isFresh = false;

                tx = Double.MAX_VALUE;
                ty = Double.MAX_VALUE;
                txnc = Double.MAX_VALUE;
                tync = Double.MAX_VALUE;
                ta = Double.MAX_VALUE;
                tid = Integer.MAX_VALUE;

                pose3d = new Pose3d(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE, new Rotation3d(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE));
                pose2d = new Pose2d(Double.MAX_VALUE, Double.MAX_VALUE, new Rotation2d(Double.MAX_VALUE));
                latency = Double.MAX_VALUE;
                timestampSeconds = Double.MAX_VALUE;
                tagCount = Integer.MAX_VALUE;
                tagSpan = Double.MAX_VALUE;
                avgTagDist = Double.MAX_VALUE;
                avgTagArea = Double.MAX_VALUE;
            }
        }
        else
        if (stats.value.length == 0)
        {
            System.out.println(name + " not connected");
        }   
    }

    /**
     * Format Target data and MegaTag2 data to print
     * 
     * @return formatted string of fresh target and MegaTag2 data or "\nno fresh data\n"
     */
    public String toString()
    {
        StringBuilder sb = new StringBuilder(900);

        if (this.isFresh())
        {
            sb.append("\n--- TARGET  STATISTICS ---\n");
            sb.append(stats.timestamp + " local time\n");
            sb.append(stats.serverTime + " server time\n");
            sb.append(stats.value[0] + " valid\n"); // useless print always 1.0 since it's a subset of the check for "isFresh"
            sb.append(stats.value[1] + " count\n");
            sb.append(stats.value[2] + " target latency\n");
            sb.append(stats.value[3] + "  capture latency\n");
            sb.append(getTX() + " horizontal offset\n");
            sb.append(getTY() + " vertical offset\n");
            sb.append(getTXNC() + "horizontal offset no crosshair\n");
            sb.append(getTYNC() + " vertical offset no crosshair\n");
            sb.append(getTA() + " area\n");
            sb.append(getTID() + " tag id\n"); 
            sb.append("\n--- MEGATAG2 POSE ---\n");
            sb.append(getTimestampSeconds() + " Time of the pose\n");
            sb.append(getPose2d() + " pose 2-D\n");
            sb.append(getPose3d() + " pose 3-D\n");
            sb.append(getLatency() + " total latency of the pose\n");
            sb.append(getTagCount() + " number of tags seen to make the pose\n");
            sb.append(getTagSpan() + " span of the tags\n");
            sb.append(getAvgTagDist() + " average distance from tags to robot\n");
            sb.append(getAvgTagArea() + " average area of the tags in their frames\n");
        }
        else
        {
            sb.append("\nno fresh data\n");
        }

        return sb.toString();
    }

  /**
   * Verify limelight name exists as a table in NT.
   * <p>
   * This check is expected to be run once during robot construction and is not intended to be checked
   * in the iterative loop.
   *
   * @param limelightName Limelight Name to check for table existence.
   * @return true if an NT table exists with requested LL name.
   * <p>false and issues a WPILib Error Alert if requested LL doesn't appear as an NT table.
   */
  @SuppressWarnings("resource")
  public static boolean isAvailable(String limelightName)
  {
    // LL sends key "getpipe" if it's on so check that
    // put in a delay if needed to help assure NT has latched onto the LL if it is transmitting
    for (int i = 1; i <= 15; i++)
    {
      if (CameraLance.NTinstance.getTable(limelightName).containsKey("getpipe"))
      {
        return true;
      }
      System.out.println("waiting " + i + " of 15 seconds for limelight named " + limelightName + " to attach");
      try
      {
        Thread.sleep((long) Seconds.of(1).in(Milliseconds));
      } catch (InterruptedException e)
      {
        e.printStackTrace();
      }
    }
    String errMsg = "Your limelight name \"" + limelightName +
                    "\" is invalid; doesn't exist on the network (no getpipe key).\n" +
                    "These may be available:> " +
                    CameraLance.NTinstance.getTable("/").getSubTables().stream()
                                        .filter(ntName -> ((String) (ntName)).startsWith("limelight"))
                                        .collect(Collectors.joining("\n")) +
                                        " < If in simulation, check LL Dashboard: Settings / Custom NT Server IP:";
    new Alert(errMsg, AlertType.kError).set(true);
    DriverStation.reportWarning(errMsg, false);
    return false;
  }
}

/*
stddevs
    doubleArray MegaTag Standard Deviations
        MT1x,
        MT1y,
        MT1z,
        MT1roll,
        MT1pitch,
        MT1Yaw,
        MT2x,
        MT2y,
        MT2z,
        MT2roll,
        MT2pitch,
        MT2yaw
    */
// stddevs = table.getDoubleArrayTopic("stddevs").subscribe(
//     new double[]{ // default stddevs huge number so as not to be used but validation should have prevented that anyway
//         9999., 9999., 9999., 9999., 9999., 9999.,
//         9999., 9999., 9999., 9999., 9999., 9999.
//     });
// private final DoubleArraySubscriber stddevs;
// var sd = stddevs.get();
// System.out.println("standard deviations - x, y, z, roll, pitch, yaw");
// System.out.format("MegaTag1 %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f%n", sd[0], sd[1], sd[2], sd[3], sd[4], sd[5]);
// System.out.format("MegaTag2 %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f%n", sd[6], sd[7], sd[8], sd[9], sd[10], sd[11]);

// example use of interpreting the JSON string from LL - it's a bit slow; not implemented here yet unless wanted
// LimelightHelpers.getLatestResults("limelight"); // 0.2 milliseconds (1 tag) to 0.3 milliseconds (2 tags), roughly
