// https://github.com/BroncBotz3481/YALL

package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.logging.Logger;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.wpilibj.Timer;

import limelight.Limelight;
import limelight.estimator.LimelightPoseEstimator;
import limelight.structures.AngularVelocity3d;

public class YALL {

  private static Logger LOGGER;
  static {
    LOGGER = Logger.getLogger("");
    LOGGER.info("Loading");     
  }

  // input from LL NT
  Limelight limelight;
  LimelightPoseEstimator limelightPoseEstimatorMT2;

  // output to NT
  // Define table for Pose3d output
  NetworkTable poseTable = NetworkTableInstance.getDefault().getTable("apriltagsYALL");
  // Create a StructTopic for a Pose3d object
  StructTopic<Pose3d> poseTopic = poseTable.getStructTopic("MyPose", Pose3d.struct);
  // Create a publisher for the topic
  StructPublisher<Pose3d> posePublisher = poseTopic.publish();

  Pose3d previousPoseEstimate = new Pose3d();
  Pose3d centerFieldPose = new Pose3d(8.27052575, 4.1055, 0., new Rotation3d(0., 0., 0.));
  Pose3d originPose = new Pose3d();


  public YALL(String name) {

    limelight = new Limelight(name);

    limelight.settingsBuilder().withPipelineIndex(0).save();

    var useMegaTag2 = true;
    limelightPoseEstimatorMT2 = limelight.getPoseEstimator(useMegaTag2);
  }

  public void LLacquire() {

    limelight.settingsBuilder().withRobotOrientation
      (new limelight.structures.Orientation3d
          (new Rotation3d(),
          new AngularVelocity3d(DegreesPerSecond.of(0.),DegreesPerSecond.of(0.),DegreesPerSecond.of(0.))
          ));

    // if (limelightPoseEstimatorMT2.getPoseEstimate().isPresent() && limelightPoseEstimatorMT2.getPoseEstimate().get().hasData)
    // {
    //     System.out.println("MT2 pose estimate\n" + limelightPoseEstimatorMT2.getPoseEstimate());
    // }

    var poseEstimate = limelightPoseEstimatorMT2.getPoseEstimate().get().pose;

    if (!limelight.getData().getResults().get().valid) // not valid iteration
    {
      System.out.println(Timer.getFPGATimestamp() + " invalid data so sitting at origin"); // useless pose; skip it
    }
    else
    if (poseEstimate.equals(centerFieldPose)) // MegaTag2 snaps to mid field if geometry is inconsistent
    {
      System.out.println(Timer.getFPGATimestamp() + " inconsistent pose so sitting at midfield"); // useless pose; skip it
    }
    else
    if (poseEstimate.equals(originPose)) // snap to origin if no data
    {
      System.out.println(Timer.getFPGATimestamp() + " no data so sitting at origin"); // useless pose; skip it
    }
    else
    if (poseEstimate.equals(previousPoseEstimate)) // no new data from camera - maybe check timestamp from camera? does that work and more efficient?
    {
      System.out.println(Timer.getFPGATimestamp() + " .......no new data");
    }
    else
    {
      System.out.println(Timer.getFPGATimestamp() + " ...................good new data");
    }

    previousPoseEstimate = poseEstimate;

    posePublisher.set(poseEstimate);
    }
}
/*
package frc.robot;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.TimedRobot;
import limelight.Limelight;
import limelight.estimator.LimelightPoseEstimator;
import limelight.structures.AngularVelocity3d;

public class Robot extends TimedRobot {

  String name = "limelight";
  Limelight limelight;
  LimelightPoseEstimator limelightPoseEstimatorMT2;
  LimelightPoseEstimator limelightPoseEstimator;

  public Robot() {
      limelight = new Limelight(name);

      var useMegaTag2 = true;
      limelightPoseEstimatorMT2 = limelight.getPoseEstimator(useMegaTag2);

      useMegaTag2 = false;
      limelightPoseEstimator = limelight.getPoseEstimator(useMegaTag2);
  }

  @Override
  public void robotPeriodic() {

    limelight.settingsBuilder().withRobotOrientation
        (
        new limelight.structures.Orientation3d
            (
            new Rotation3d(),
            new AngularVelocity3d(DegreesPerSecond.of(0.),DegreesPerSecond.of(0.),DegreesPerSecond.of(0.))
            )
        );


// PICK YOUR TESTS TO RUN
//                            0       1     2       3       4       5  
    var tests = new boolean[]{false, true, false, false, false, false};

/**
 * Caution -- test case restrictions:
 * I have some ".orElseThrow" which crashes the robot on no data. Yes, that's what I wanted to test at the time.
 * More realistic examples are checking for ".isPresent()".

//  if (tests[0])
//  {
//      System.out.println("2d target status (validity Tv) " + limelight.getData().targetData.getTargetStatus());
//      for (limelight.results.RawFiducial tag : limelight.getData().getRawFiducials()) // tested okay; insensitive to LL target grouping; returns all targets
//      { 
//      System.out.println(tag);
//      }      
//  }

//  if (tests[1])
//  {
//      if (limelight.getLatestResults().isPresent())
//      {
//          System.out.println(limelight.getLatestResults());
//      }
//  }

//  if (tests[2])
//  {
//      if (limelightPoseEstimatorMT2.getPoseEstimate().isPresent() && limelightPoseEstimatorMT2.getPoseEstimate().get().hasData)
//      {
//          System.out.println("MT2 pose estimate\n" + limelightPoseEstimatorMT2.getPoseEstimate());
//      }
//  }

//  if (tests[3])
//  {
//      System.out.println("MT2 alliance pose estimate\n" + limelightPoseEstimatorMT2.getAlliancePoseEstimate().orElseThrow());
//  }

//  if (tests[4])
//  {
//      System.out.println("pose estimate\n" + limelightPoseEstimator.getPoseEstimate().orElseThrow());
//  }

//  if (tests[5])
//  {
//      System.out.println("alliance pose estimate\n" +limelightPoseEstimator.getAlliancePoseEstimate().orElseThrow());
//  }

// }

// @Override
// public void autonomousInit() {limelight.settingsBuilder().withPipelineIndex(0);}

// @Override
// public void teleopInit() {limelight.settingsBuilder().withPipelineIndex(1);
//  // limelight.snapshot("RickTestToDelete"); // couldn't get this to work nor get LL to manage snapshots like it's supposed to
// }

// Example 1: Classifier Target Tracking
// Limelight limelight = new Limelight("limelight");
// // Get the results
// limelight.getLatestResults().ifPresent((LimelightResults result) -> {
//     for (NeuralClassifier object : result.targets_Classifier)
//     {
//         // Classifier says its a coral.
//         if (object.className.equals("coral"))
//         {
//             // Check pixel location of coral.
//             if (object.ty > 2 && object.ty < 1)
//             {
//             // Coral is valid! do stuff!
//             }
//         }
//     }
// });

// Example 2: Vision Pose Estimation with MegaTag2
// Limelight limelight = new Limelight("limelight");

// // Required for megatag2 in periodic() function before fetching pose.
// limelight.getSettings()
// 		 .withRobotOrientation(new Orientation3d(gyro.getRotation3d(),
// 												 new AngularVelocity3d(DegreesPerSecond.of(gyro.getPitchVelocity()),
// 																	   DegreesPerSecond.of(gyro.getRollVelocity()),
// 																	   DegreesPerSecond.of(gyro.getYawVelocity()))))
// 		 .save();
  
// // Get MegaTag2 pose
// Optional<PoseEstimate> visionEstimate = poseEstimator.getPoseEstimate();
// // If the pose is present
// visionEstimate.ifPresent((PoseEstimate poseEstimate) -> {
//     // Add it to the pose estimator.
//     poseEstimator.addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds);
// });

*/
