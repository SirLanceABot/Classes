package frc.robot;

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
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
 
public class Camera4237 {

    private final String name;
    private final DoubleArraySubscriber t2d;
    private final DoubleArraySubscriber botpose_orb_wpiblue;
    private final DoubleArraySubscriber stddevs;
    private final DoubleArrayPublisher robot_orientation_set;
    private final DoublePublisher stream;
    private final StructPublisher<Pose3d> botpose_orb_wpiblueAS;
    private Pose3d robotPoseBlueMT2 = new Pose3d(); // initialize in case it's used before being set (not supposed to happen)

    public Camera4237(String name) {

        if (!isAvailable(name))
        {
          throw new RuntimeException("Limelight named " + name + " is not available");
        }

        this.name = name;

        // Get the default instance of NetworkTables
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        // Get the limelight table
        var table = inst.getTable(name);

        // Get the AdvantageScope table
        var AStable = inst.getTable(name + "AS");

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
        stddevs = table.getDoubleArrayTopic("stddevs").subscribe(
            new double[]{ // default stddevs huge number so as not to be used but validation should have prevented that anyway
                9999., 9999., 9999., 9999., 9999., 9999.,
                9999., 9999., 9999., 9999., 9999., 9999.
            });

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

        // LL pose formatted to publish to AdvantageScope
        botpose_orb_wpiblueAS = AStable.getStructTopic("botpose_orb_wpiblueAS", Pose3d.struct).publish();
    }

    /**
     * Enables standard side-by-side stream mode.
     */
    public void setStreamMode_Standard(String limelightName) {
        stream.set(0.);
    }

    /**
     * Enables Picture-in-Picture mode with secondary stream in the corner.
     */
    public void setStreamMode_PiPMain(String limelightName) {
        stream.set(1.);
    }

    /**
     * Enables Picture-in-Picture mode with primary stream in the corner.
     */
    public void setStreamMode_PiPSecondary() {
        stream.set(2.);
    }

    /**
     * Publish the robot orientation
     * Read the latest values from the subscribers
     * Call this method in the robot iterative loop
     */
    public void update() {

        // for MegaTag2 set the robot orientation from the gyro heading and rate.
        //FIXME get the gyro values somehow but here is zeros for test data - limits what AprilTags make sense
        var gyro = new double[]{0., 0., 0., 0., 0., 0.};
        robot_orientation_set.set(gyro); // robot orientation sent to LL for MegaTag2

        // get LL data needed to determine validity
        var stats = t2d.getAtomic();
        var valid = 17 == stats.value.length && 1.0 == stats.value[0];

        if (valid)
        {
            var pose = botpose_orb_wpiblue.getAtomic(); // get the LL MegaTag2 pose data
            robotPoseBlueMT2 = new Pose3d(pose.value[0], pose.value[1], pose.value[2], new Rotation3d(pose.value[3], pose.value[4], pose.value[5])); // robot in field 3d pose
            var poseForAddVisionMeasurement = new Pose2d(pose.value[0], pose.value[1], new Rotation2d(Units.degreesToRadians(pose.value[5]))); // robot in field 2d pose

            // timestamp of data for pose estimation
            long timestampSeconds = pose.timestamp;
            double latency = pose.value[6];
            double adjustedTimestampSeconds = (timestampSeconds / 1000000.0) - (latency / 1000.0); // Convert server timestamp from microseconds to seconds and adjust for latency
            // data usage for pose estimation
            // m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999)); //FIXME need stddev tuning/filtering
            // m_poseEstimator.addVisionMeasurement(poseForAddVisionMeasurement, adjustedTimestampSeconds);

            System.out.println("local time " + stats.timestamp + ", server time " + stats.serverTime
            + ", valid " + stats.value[0] + ", count " + stats.value[1] + " total latency " + (stats.value[2]+stats.value[3])
            + ", horizontal offset " + stats.value[4] + " tag id " + stats.value[9] + " ,skew deg " +stats.value[16]);
            System.out.println("addVisionMeasurement Pose2d " + poseForAddVisionMeasurement + ", timestamp " + adjustedTimestampSeconds);
            System.out.println(robotPoseBlueMT2);
            System.out.println("total latency " + pose.value[6] + ", count " + pose.value[7] + ", span " + pose.value[8] + ", average distance " + pose.value[9] + ", average area " + pose.value[10]);
// addVisionMeasurement Pose2d Pose2d(Translation2d(X: 11.81, Y: 4.04), Rotation2d(Rads: 0.00, Deg: 0.00)), timestamp 97.64429015905762
// local time 171282476, server time 171282476, valid 1.0, count 2.0 total latency 174.8796157836914, horizontal offset 10.477392196655273 tag id 12.0 ,skew deg 75.9100570678711
// Pose3d(Translation3d(X: 1.26, Y: -0.07, Z: 0.00), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0)))
// total latency 174.87960815429688, count 2.0, span 0.335460891519132, average distance 0.7605234507148955, average area 4.441176541149616
            var sd = stddevs.get();
            System.out.println("standard deviations - x, y, z, roll, pitch, yaw");
            System.out.format("MegaTag1 %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f%n", sd[0], sd[1], sd[2], sd[3], sd[4], sd[5]);
            System.out.format("MegaTag2 %6.2f, %6.2f, %6.2f, %6.2f, %6.2f, %6.2f%n", sd[6], sd[7], sd[8], sd[9], sd[10], sd[11]);
// standard deviations - x, y, z, roll, pitch, yaw
// MegaTag1   3.48,   0.56,   0.53,  18.04,   8.94,  66.13
// MegaTag2   0.12,   0.03,   0.00,   0.00,   0.00,   0.00

            poseToAS();
        }
        else
        if (stats.value.length == 0)
        {
            System.out.println(name + " not connected");
        }   
    }

    public String toString()
    {
        return null;
    }

    /**
     * Publish the limelight 3d pose to NT in AdvantageScope format
     * for MegaTag2 blue
     */
    public void poseToAS()
    {
        botpose_orb_wpiblueAS.set(robotPoseBlueMT2);
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
      if (NetworkTableInstance.getDefault().getTable(limelightName).containsKey("getpipe"))
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
                    "These may be available:" +
                    NetworkTableInstance.getDefault().getTable("/").getSubTables().stream()
                                        .filter(ntName -> ((String) (ntName)).startsWith("limelight"))
                                        .collect(Collectors.joining("\n"));
    new Alert(errMsg, AlertType.kError).set(true);
    return false;
  }
}
