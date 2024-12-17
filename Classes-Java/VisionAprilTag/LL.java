// 2023 usage; not reviewed for 2024
package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.logging.Logger;
import java.util.stream.Collectors;

public class LL {

  private static Logger LOGGER;
  static {
    LOGGER = Logger.getLogger("");
    LOGGER.info("Loading");     
  }

  int maxTagId = 25    +1; // add 1 for tag ID 0

  NetworkTable tagsTable = NetworkTableInstance.getDefault().getTable("apriltagsLL");

  List<StructPublisher<Pose3d>> publishRobotPoseLL = new ArrayList<>(maxTagId);
  List<StructPublisher<Pose3d>> publishRobotPoseBlueLL = new ArrayList<>(maxTagId);

  double poseLL5Prev = 0.;
  double noLL = Double.NaN;
  double validLL = 1.;
  double noTagLL = -1.;

  public LL() {
    // make an empty bucket for every possible tag
    for (int tag = 0; tag < maxTagId; tag++) {
      publishRobotPoseLL.add(null);
      var robotPosePublisher = tagsTable.getStructTopic("robotPose3D_" + tag, Pose3d.struct).publish();
      publishRobotPoseLL.set(tag, robotPosePublisher);

      publishRobotPoseBlueLL.add(null);
      var robotPoseBluePublisher = tagsTable.getStructTopic("robotPose3DBlue_" + tag, Pose3d.struct).publish();
      publishRobotPoseBlueLL.set(tag, robotPoseBluePublisher);
    }
  }

  public void LLacquire() {

    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
    NetworkTableEntry tv = table.getEntry("tv");
    NetworkTableEntry ts = table.getEntry("ts");

    //read values periodically
    double x = tx.getDouble(noLL);
    double y = ty.getDouble(noLL);
    double area = ta.getDouble(noLL);
    double valid = tv.getDouble(noLL);
    double skew = ts.getDouble(noLL);

    //post to smart dashboard periodically
    SmartDashboard.putNumber("LimelightX", x);
    SmartDashboard.putNumber("LimelightY", y);
    SmartDashboard.putNumber("LimelightArea", area);
    SmartDashboard.putNumber("LimelightValid", valid);
    SmartDashboard.putNumber("LimelightSkew", skew);

    var
    tid = NetworkTableInstance
              .getDefault()
              .getTable("limelight")
              .getEntry("tid")
              .getDouble(noLL);
    
    var
    poseLL = NetworkTableInstance
              .getDefault()
              .getTable("limelight")
              .getEntry("botpose_wpiblue")
              .getDoubleArray(new double[]{noLL, noLL, noLL, noLL, noLL, noLL, noLL});

    // skip invalid or unchanging data 
    if(valid != validLL || tid == noTagLL || tid == noLL || poseLL[5] == poseLL5Prev)
    {
      SmartDashboard.putString("LL", "no target");
      return; 
    }
    else {
      SmartDashboard.putString("LL", "valid target");
    }

    poseLL5Prev = poseLL[5];

    // System.out.println("botpose_wpiblue\n" + Arrays.toString(poseLL));

    // botpose_wpiblue	Robot transform in field-space (blue driverstation WPILIB origin).
    // Translation (X,Y,Z)[meters] Rotation(Roll,Pitch,Yaw)[degrees], total latency (cl+tl)
    var // convert LL rotation degrees format to AdvantageScope rotation quaternion format and re-post so LL
        // can display on AdvantageScope 3D Field tab
    robotInField = new Pose3d(
                new Translation3d(poseLL[0], poseLL[1], poseLL[2]),
                  new Rotation3d(
                              Units.degreesToRadians(poseLL[3]),
                            Units.degreesToRadians(poseLL[4]),
                              Units.degreesToRadians(poseLL[5])) );

    var
    robotPoseBlue = LimelightHelpers.getLatestResults("limelight").getBotPose3d_wpiBlue();

    // put out to NetworkTables this tag's robot pose
    publishRobotPoseLL.get((int)tid).set(robotInField);
    publishRobotPoseBlueLL.get((int)tid).set(robotPoseBlue);
  }

  String toString(double[] array) {
    return Arrays.stream(array)
            .mapToObj(i -> String.format("%5.2f", i))
           // .collect(Collectors.joining(", ", "[", "]"));
            .collect(Collectors.joining("|", "|", "|"));
  }
}
/*
"{\"Results\":
  {\"Bardcode\":[],\"Classifier\":[],\"Detector\":[],
    \"Fiducial\":[{\"fID\":8,\"fam\":\"16H5C\",\"pts\":[],\"skew\":[],
    \"t6c_ts\":[0.2979426951937259,0.13623731786903492,-0.7375089362296684,13.766155220250653,-18.060270182834813,4.196460593074876],
    \"t6r_fs\":[-6.505590668461496,-2.638648283329339,0.3264826821309651,4.130892500067147,-15.031855952512505,-162.96762048338223],
    \"t6r_ts\":[0.2979426951937259,0.13623731786903492,-0.7375089362296684,13.766155220250653,-18.060270182834813,4.196460593074876],
    \"t6t_cs\":[-0.06334161310114575,0.07871411866347018,0.8006508695997557,-15.688162205795916,16.432863211965326,-8.673794516483635],
    \"t6t_rs\":[-0.06334161310114575,0.07871411866347018,0.8006508695997557,-15.688162205795916,16.432863211965326,-8.673794516483635],
    \"ta\":0.030110761523246765,\"tx\":-6.159928321838379,\"txp\":263.4709777832031,\"ty\":-5.092217445373535,\"typ\":285.678955078125}],
    \"Retro\":[],\"botpose\":[-6.50559135673696,-2.63864700363727,0.3264828203893591,4.130891665477431,-15.03184609377145,-162.96752381854927],
    \"botpose_wpiblue\":[1.7652836432630403,1.3682029963627302,0.3264828203893591,4.130891665477431,-15.03184609377145,-162.96752381854927],
    \"botpose_wpired\":[14.776473358623518,6.6454797404669375,0.3264828203893591,4.130891665477431,-15.03184609377145,17.032324141959332],
    \"cl\":16.275556564331055,\"pID\":0.0,\"t6c_rs\":[0.0,0.0,0.0,0.0,0.0,0.0],\"tl\":65.90687561035156,\"ts\":25570118.558786,\"v\":1}}"
*/

