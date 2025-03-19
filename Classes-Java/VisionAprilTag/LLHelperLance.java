package frc.robot;

import static edu.wpi.first.units.Units.Milliseconds;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.TimestampedDoubleArray;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;
import java.util.logging.Logger;
import java.util.stream.Collectors;

/**
 * Wrapper class for LimelightHelpers and other suggested usage helpers.
 * <p>Creates a robot pose (2d or the equivalent 3d) from valid data using
 * either the MegaTag for a close-in tag view or MegaTag2 for more distant
 * views.
 * <p>Poses are sent to NetworkTables for examples.
 * <p>Since MegaTag2 is used beyond 1.1 meters the robot orientation (gyro) must be given to LL.
 <pre>
// Construct an instance once (for each Limelight camera)
    LLHelpers = useLLHelpers ? new LLHelperLance(limelightName) : null;
 
// Periodically (for each Limelight camera)
    if (LLHelpers != null)
    {
      LimelightHelpers.SetRobotOrientation(limelightName, 0., 0., 0., 0., 0., 0.); // fake test data good for reef 10 & 18

      LLHelpers.LLacquire();
      
      if(LLHelpers.isFresh())
      {
        System.out.println(getTagId() + ", " + getRobotInField() + ", " + LLHelpers);
        // or use the identical data from getRobotInField3d());
        if(isSuggestedResetOdometry())
        {
          drivetrain.resetOdometry(getRobotInField());
        }
        else
        {
          drivetrain.addVisionMeasurement(getRobotInField());
        }
      }
    }
 </pre>
 */

@Logged
public class LLHelperLance {

  private static Logger LOGGER;
  static {
    LOGGER = Logger.getLogger("");
    LOGGER.info("Loading");     
  }

  String LLname;
  private static final NetworkTableInstance NTinstance = NetworkTableInstance.getDefault();
  private NetworkTable LLTableIn;
  private final DoubleArraySubscriber t2d;
  StructPublisher<Pose2d> publishRobotPoseMT1;
  StructPublisher<Pose2d> publishRobotPoseMT2;
  StructPublisher<Pose2d> publishRobotPose;

  StructPublisher<Pose3d> publishRobotPoseBlueJSON;
  private boolean suggestResetOdometry = false; // indicates excellent LL pose estimation; initialize in case used before an acquire
  private boolean LLavailable;
  // fields associated with statistics and target    
  private TimestampedDoubleArray stats;
  // private double tx;
  // private double ty;
  // private double ta;
  private int tid;
  private boolean isFresh;
  private long previousTimestamp = 0; // arbitrary initial time to start
  private Pose2d robotInField = Pose2d.kZero;

  public LLHelperLance(String LLname) {
    this.LLname = LLname;
    LLTableIn = NTinstance.getTable(LLname);
    LLavailable = isAvailable();
    final NetworkTable LLTableOut = NTinstance.getTable("LLLance"+LLname);  // Get the limelight table

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
    t2d = LLTableIn.getDoubleArrayTopic("t2d").subscribe(new double[]{}); // default is no data (array length = 0)

    publishRobotPoseMT1 = LLTableOut.getStructTopic("robotPose2DMT1", Pose2d.struct).publish();
    publishRobotPoseMT2 = LLTableOut.getStructTopic("robotPose2DMT2", Pose2d.struct).publish();
    publishRobotPose = LLTableOut.getStructTopic("robotPose2D", Pose2d.struct).publish();
  }

  public void LLacquire() {

    // initialize each iteration to origin or something invalid for anything that could be returned to user
    // invalid data leaves the previously published poses unchanged but the poses are zeros if used
    var robotInFieldMT1 = Pose2d.kZero;
    var robotInFieldMT2 = Pose2d.kZero;
    robotInField = Pose2d.kZero;
    tid = -1;
    suggestResetOdometry = false; // believe pose is best estimate if true

    // get LL data needed to determine validity and more
    stats = t2d.getAtomic(); // mostly duplicates LLHelpers but I wanted the timestamp to check for updated

    // check if new data
    isFresh = stats.timestamp != previousTimestamp && stats.value.length >= 17 && stats.value[0] == 1.0 && LLavailable;
    previousTimestamp = stats.timestamp;

    // skip invalid or unchanging data
    if( !isFresh())
    {
      SmartDashboard.putString(LLname, "no target");
    }
    else
    {
      tid = (int)stats.value[9];
      SmartDashboard.putString(LLname, "valid target");

      // MEGATAG1 process
      var MT1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(LLname);
      robotInFieldMT1 = MT1.pose;
      publishRobotPoseMT1.set(robotInFieldMT1);

      // MEGATAG2 process
      var MT2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LLname); 
      robotInFieldMT2 = MT2.pose;
      publishRobotPoseMT2.set(robotInFieldMT2);

      // MT1 works better than MT2 close in.
      // MT1 is too jittery beyond 1.1 meters and MT2 isn't as accurate less than 1 meter
      // MT2 quits when close to target and MT1 keeps on working
      // MT2 lag for gyro may be tricky to get right
      // MT2 goes to pose at field center when it's invalid - not checking for that - keeping the faith
      if (MT1.tagCount == 1 && MT1.avgTagDist <= 1.1)
      {
        robotInField = MT1.pose;
        suggestResetOdometry = true;
      }
      else
      {
        robotInField = MT2.pose;
      }

      publishRobotPose.set(robotInField);
        
      // // testing - post to smart dashboard periodically
      // tx = stats.value[4];
      // ty = stats.value[5];
      // ta = stats.value[8];
      // SmartDashboard.putNumber("LimelightX", tx);
      // SmartDashboard.putNumber("LimelightY", ty);
      // SmartDashboard.putNumber("LimelightArea", ta);      
    }

    // // testing JSON results - must be activated on the LL dashboard
    // var
    // JSONdump = LimelightHelpers.getLatestResults(name);
    // System.out.println(JSONdump);
  }

  /**
   * New, valid pose data
   * @return true if data are useful; false if data is not to be used
   */
  public boolean isFresh()
  {
    return isFresh;
  }

  /**
   * Closest tag seen for at this pose
   * @return
   */
  public int getTagId()
  {
    return tid;
  }

  /**
   * Robot Pose
   * @return the 2d pose
   */
  public Pose2d getRobotInField()
  {
    return robotInField;
  }

  /** Convenience method that returns the current 2d pose as a 3d pose
   * @return the 2d pose as a 3d pose
   */
  public Pose3d getRobotInField3d()
  {
    return new Pose3d(robotInField);
  }
  /**
   * 
   * Recommended to fully trust LL pose if everything is valid and MegaTag1 is used within 1.1 meters of the tag
   * @return true if recommended to reset odometry to the LL pose
   * @see {@link #getTagId} to see closest tag
   */
  public boolean isSuggestResetOdometry()
  {
      return suggestResetOdometry;
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
  public boolean isAvailable()
  {
    // LL sends key "getpipe" if it's on so check that
    // put in a delay if needed to help assure NT has latched onto the LL if it is transmitting
    for (int i = 1; i <= 15; i++)
    {
      if (LLTableIn.containsKey("getpipe"))
      {
        return true;
      }
      System.out.println("waiting " + i + " of 15 seconds for limelight named " + LLname + " to attach");
      try
      {
        Thread.sleep((long) Seconds.of(1).in(Milliseconds));
      } catch (InterruptedException e)
      {
        e.printStackTrace();
      }
    }
    String errMsg = "Your limelight name \"" + LLname +
                    "\" is invalid; doesn't exist on the network (no getpipe key).\n" +
                    "These may be available:> " +
                    NTinstance.getTable("/").getSubTables().stream()
                                        .filter(ntName -> ((String) (ntName)).startsWith("limelight"))
                                        .collect(Collectors.joining("\n")) +
                                        " < If in simulation, check LL Dashboard: Settings / Custom NT Server IP:";
    new Alert(errMsg, AlertType.kError).set(true);
    DriverStation.reportWarning(errMsg, false);
    return false;
  }

  /**
   * This class toString
   * @return minimal information
   */
  public String toString()
  {
    return "Fresh data " + isFresh() + ", suggest reset odometry " + isSuggestResetOdometry() + ", robot pose " + getRobotInField() + ", nearest tag " + getTagId();
  }

  String toString(double[] array) {
    return Arrays.stream(array)
            .mapToObj(i -> String.format("%5.2f", i))
           // .collect(Collectors.joining(", ", "[", "]"));
            .collect(Collectors.joining("|", "|", "|"));
  }
}
