// BetaBot2025-develop[ElectroBunny].zip
// 3 PIDs to align to reef

package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private Timer dontSeeTagTimer, stopTimer;
  private double tagID = -1;
  private final Pose2d targetBranch;

  public AlignToReefTagRelative(boolean isRightScore) {
    xController = new PIDController(X_REEF_ALIGNMENT_KP, 0.0, 0);  // Vertical movement
    yController = new PIDController(Y_REEF_ALIGNMENT_KP, 0.0, 0);  // Horizontal movement
    rotController = new PIDController(ROT_REEF_ALIGNMENT_KP, 0, 0);  // Rotation
    if (isRightScore)
    {
        targetBranch = new Pose2d(-0.65, -0.17, new Rotation2d(0.));
    }
    else
    {
        targetBranch = new Pose2d(-0.65, 0.17, new Rotation2d(0.));
    }

  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    xController.setSetpoint(targetBranch.getX());
    xController.setTolerance(X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(targetBranch.getY());
    yController.setTolerance(Y_TOLERANCE_REEF_ALIGNMENT);

    rotController.setSetpoint(targetBranch.getRotation().getRadians()); // FIXME what units are needed for drive?
    rotController.setTolerance(ROT_TOLERANCE_REEF_ALIGNMENT);

    //FIXME what to do if tag isn't seen at the instant operator wants auto targeting and this commend is initialized?
    tagID = LimelightHelpers.getFiducialID("");
    tagID = 10;
    System.out.println("Align to reef command initialized");
  }

  @Override
  public void execute() {
    // FIXME check if same tag as started with? Use robot pose Estimator between valid tag readings

    var robotPoseTagRelativeNWU = getRobotPose();
    double xSpeed = xController.calculate(robotPoseTagRelativeNWU.getX());
    double ySpeed = -yController.calculate(robotPoseTagRelativeNWU.getY());
    double rotValue = -rotController.calculate(robotPoseTagRelativeNWU.getRotation().getRadians());

    drive(xSpeed, ySpeed, rotValue, false); // if chassis speeds needed, then use something like new ChassisSpeeds(xSpeed, ySpeed, rotValue)

    //FIXME "if" statements belong in "isFinished"
    if (!rotController.atSetpoint() ||
        !yController.atSetpoint() ||
        !xController.atSetpoint()) {
    stopTimer.reset();
    }
    else {
      drive(0, 0, 0, false);
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drive(0, 0, 0, false);
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long as it gets a tag in the camera
    // Is this needed or realistic?
    return this.dontSeeTagTimer.hasElapsed(DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(POSE_VALIDATION_TIME);
  }

  	// Auto constants
	public static final double X_REEF_ALIGNMENT_KP = 3.3;
	public static final double Y_REEF_ALIGNMENT_KP = 3.3;
	public static final double ROT_REEF_ALIGNMENT_KP = 0.058;

	public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.02; // meters
	public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02; // meters
	public static final double ROT_TOLERANCE_REEF_ALIGNMENT = Units.degreesToRadians(1.5);

	public static final double DONT_SEE_TAG_WAIT_TIME = 1;
	public static final double POSE_VALIDATION_TIME = 0.3;

    private void drive(double Tx, double Ty, double Rz, boolean fieldCentric)
    {
        System.out.println("drive speeds " +Tx + ", " + Ty + ", " + Rz);
    }

    Pose2d lastPoseFromTag = Pose2d.kZero;

    private Pose2d getRobotPose()
    {
        if (LimelightHelpers.getTV("") && LimelightHelpers.getFiducialID("") == tagID) {
            this.dontSeeTagTimer.reset();
            var tagInRobotFrameEDN = LimelightHelpers.getTargetPose3d_RobotSpace("");
            var tagInRobotFrameTemp = CoordinateSystem.convert(tagInRobotFrameEDN, CoordinateSystem.EDN(), CoordinateSystem.NWU());
    
            // adjust rotations to come out right - why didn't they convert right? Because we shouldn't do it this way?
            // this is target facing the robot and moving wrt to robot in normal WPILib field orientations
            // but the target moving wrt the robot is NOT normal thinking and the tag facing the robot should be
            // indicated by a Pi rotation. All this trickery is confusing - don't do it this way!
            var tagInRobotFrame = new Pose3d(
            tagInRobotFrameTemp.getTranslation(),
            new Rotation3d(
                tagInRobotFrameTemp.getRotation().getY(), // flip X and Y but why?
                -tagInRobotFrameTemp.getRotation().getX() - Math.PI/2.,
                tagInRobotFrameTemp.getRotation().getZ() +
                (tagInRobotFrameTemp.getRotation().getZ() < Math.PI/2. ? - Math.PI - Math.PI/2. : 0.0)));
            var tagInRobotFrame2D = new Pose2d( tagInRobotFrame.getX(), tagInRobotFrame.getY(), new Rotation2d(tagInRobotFrame.getRotation().getZ()));
            var robotPoseTagRelativeNWU = new Pose2d(-tagInRobotFrame2D.getX(), -tagInRobotFrame2D.getY(), tagInRobotFrame2D.getRotation().unaryMinus());
            lastPoseFromTag = robotPoseTagRelativeNWU;
            return robotPoseTagRelativeNWU;
        }
        else return lastPoseFromTag; //FIXME better to get the poseEstimator or odometry if no tag update
    }
}
/*
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.12), Rotation2d(Rads: 0.06, Deg: 3.66))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.11), Rotation2d(Rads: 0.06, Deg: 3.65))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.12), Rotation2d(Rads: 0.06, Deg: 3.29))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.12), Rotation2d(Rads: 0.07, Deg: 3.80))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.12), Rotation2d(Rads: 0.08, Deg: 4.55))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.12), Rotation2d(Rads: 0.07, Deg: 4.10))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.12), Rotation2d(Rads: 0.08, Deg: 4.62))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.12), Rotation2d(Rads: 0.07, Deg: 4.19))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.13), Rotation2d(Rads: 0.08, Deg: 4.42))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.07, Deg: 3.80))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.14), Rotation2d(Rads: 0.06, Deg: 3.62))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.14), Rotation2d(Rads: 0.08, Deg: 4.31))
robot pose to target NWU Pose2d(Translation2d(X: -0.68, Y: -0.11), Rotation2d(Rads: 0.10, Deg: 5.97))
robot pose to target NWU Pose2d(Translation2d(X: -0.68, Y: -0.08), Rotation2d(Rads: 0.16, Deg: 8.96))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.07), Rotation2d(Rads: 0.18, Deg: 10.21))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.08), Rotation2d(Rads: 0.18, Deg: 10.44))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.12), Rotation2d(Rads: 0.16, Deg: 8.95))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.14), Rotation2d(Rads: 0.12, Deg: 6.71))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.14), Rotation2d(Rads: 0.10, Deg: 5.64))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.10), Rotation2d(Rads: 0.13, Deg: 7.39))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.10), Rotation2d(Rads: 0.14, Deg: 7.80))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.11), Rotation2d(Rads: 0.14, Deg: 8.28))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.11, Deg: 6.26))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.09, Deg: 5.06))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.16), Rotation2d(Rads: 0.09, Deg: 5.29))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.17), Rotation2d(Rads: 0.09, Deg: 5.42))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.17), Rotation2d(Rads: 0.09, Deg: 5.15))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.10, Deg: 5.69))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.12, Deg: 6.95))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.11, Deg: 6.41))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.12, Deg: 6.63))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.15), Rotation2d(Rads: 0.12, Deg: 6.82))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.17), Rotation2d(Rads: 0.10, Deg: 5.61))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.10, Deg: 5.93))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.16), Rotation2d(Rads: 0.10, Deg: 5.60))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.18), Rotation2d(Rads: 0.09, Deg: 5.37))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.18), Rotation2d(Rads: 0.08, Deg: 4.52))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.18), Rotation2d(Rads: 0.08, Deg: 4.82))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.18), Rotation2d(Rads: 0.09, Deg: 4.94))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.18), Rotation2d(Rads: 0.09, Deg: 5.12))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.12, Deg: 6.81))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.17), Rotation2d(Rads: 0.11, Deg: 6.59))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.18), Rotation2d(Rads: 0.08, Deg: 4.66))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.18), Rotation2d(Rads: 0.08, Deg: 4.78))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.19), Rotation2d(Rads: 0.08, Deg: 4.36))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.18), Rotation2d(Rads: 0.09, Deg: 4.97))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.16), Rotation2d(Rads: 0.10, Deg: 5.56))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.16), Rotation2d(Rads: 0.10, Deg: 5.84))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.15), Rotation2d(Rads: 0.09, Deg: 5.40))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.15), Rotation2d(Rads: 0.10, Deg: 5.71))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.16), Rotation2d(Rads: 0.10, Deg: 5.77))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.16), Rotation2d(Rads: 0.10, Deg: 5.58))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.16), Rotation2d(Rads: 0.09, Deg: 5.08))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.17), Rotation2d(Rads: 0.08, Deg: 4.57))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.18), Rotation2d(Rads: 0.07, Deg: 3.90))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.18), Rotation2d(Rads: 0.06, Deg: 3.71))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.18), Rotation2d(Rads: 0.08, Deg: 4.39))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.18), Rotation2d(Rads: 0.06, Deg: 3.55))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.18), Rotation2d(Rads: 0.07, Deg: 4.02))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.18), Rotation2d(Rads: 0.07, Deg: 4.07))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.18), Rotation2d(Rads: 0.07, Deg: 3.98))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.18), Rotation2d(Rads: 0.09, Deg: 5.12))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.18), Rotation2d(Rads: 0.07, Deg: 4.02))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.18), Rotation2d(Rads: 0.08, Deg: 4.50))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.18), Rotation2d(Rads: 0.08, Deg: 4.45))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.17), Rotation2d(Rads: 0.08, Deg: 4.63))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.17), Rotation2d(Rads: 0.09, Deg: 5.04))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.16), Rotation2d(Rads: 0.09, Deg: 5.35))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.16), Rotation2d(Rads: 0.09, Deg: 5.29))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.16), Rotation2d(Rads: 0.10, Deg: 5.59))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.17), Rotation2d(Rads: 0.09, Deg: 5.32))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.17), Rotation2d(Rads: 0.09, Deg: 4.92))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.17), Rotation2d(Rads: 0.10, Deg: 5.50))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.16), Rotation2d(Rads: 0.10, Deg: 5.72))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.17), Rotation2d(Rads: 0.08, Deg: 4.73))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.17), Rotation2d(Rads: 0.08, Deg: 4.33))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.17), Rotation2d(Rads: 0.09, Deg: 5.30))
robot pose to target NWU Pose2d(Translation2d(X: -0.63, Y: -0.17), Rotation2d(Rads: 0.09, Deg: 5.42))
robot pose to target NWU Pose2d(Translation2d(X: -0.61, Y: -0.16), Rotation2d(Rads: 0.10, Deg: 5.95))
robot pose to target NWU Pose2d(Translation2d(X: -0.60, Y: -0.14), Rotation2d(Rads: 0.12, Deg: 6.89))
robot pose to target NWU Pose2d(Translation2d(X: -0.59, Y: -0.14), Rotation2d(Rads: 0.13, Deg: 7.35))
robot pose to target NWU Pose2d(Translation2d(X: -0.60, Y: -0.14), Rotation2d(Rads: 0.13, Deg: 7.38))
robot pose to target NWU Pose2d(Translation2d(X: -0.60, Y: -0.13), Rotation2d(Rads: 0.13, Deg: 7.46))
robot pose to target NWU Pose2d(Translation2d(X: -0.61, Y: -0.13), Rotation2d(Rads: 0.13, Deg: 7.39))
robot pose to target NWU Pose2d(Translation2d(X: -0.61, Y: -0.14), Rotation2d(Rads: 0.12, Deg: 7.08))
robot pose to target NWU Pose2d(Translation2d(X: -0.62, Y: -0.14), Rotation2d(Rads: 0.12, Deg: 6.98))
robot pose to target NWU Pose2d(Translation2d(X: -0.63, Y: -0.14), Rotation2d(Rads: 0.11, Deg: 6.49))
robot pose to target NWU Pose2d(Translation2d(X: -0.64, Y: -0.15), Rotation2d(Rads: 0.10, Deg: 5.78))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.17), Rotation2d(Rads: 0.09, Deg: 4.96))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.18), Rotation2d(Rads: 0.08, Deg: 4.42))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.17), Rotation2d(Rads: 0.08, Deg: 4.51))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.17), Rotation2d(Rads: 0.09, Deg: 4.87))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.17), Rotation2d(Rads: 0.09, Deg: 4.88))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.17), Rotation2d(Rads: 0.08, Deg: 4.87))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.17), Rotation2d(Rads: 0.09, Deg: 4.99))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.17), Rotation2d(Rads: 0.09, Deg: 5.15))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.17), Rotation2d(Rads: 0.09, Deg: 4.93))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.17), Rotation2d(Rads: 0.08, Deg: 4.80))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.17), Rotation2d(Rads: 0.09, Deg: 5.01))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.10, Deg: 5.45))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.16), Rotation2d(Rads: 0.10, Deg: 5.98))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.15), Rotation2d(Rads: 0.09, Deg: 5.42))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.15), Rotation2d(Rads: 0.10, Deg: 5.81))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.15), Rotation2d(Rads: 0.11, Deg: 6.21))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.15), Rotation2d(Rads: 0.10, Deg: 5.83))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.10, Deg: 5.68))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.10, Deg: 5.81))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.10, Deg: 5.75))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.10, Deg: 6.01))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.10, Deg: 5.95))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.11, Deg: 6.27))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.10, Deg: 5.69))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.11, Deg: 6.24))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.10, Deg: 5.74))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.10, Deg: 5.88))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.10, Deg: 5.51))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.10, Deg: 5.50))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.09, Deg: 5.33))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.10, Deg: 5.61))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.09, Deg: 5.44))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.09, Deg: 5.18))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.09, Deg: 5.32))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.09, Deg: 5.06))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.17), Rotation2d(Rads: 0.09, Deg: 4.93))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.18), Rotation2d(Rads: 0.08, Deg: 4.65))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.18), Rotation2d(Rads: 0.07, Deg: 4.20))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.18), Rotation2d(Rads: 0.08, Deg: 4.59))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.17), Rotation2d(Rads: 0.09, Deg: 4.99))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.17), Rotation2d(Rads: 0.09, Deg: 4.97))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.17), Rotation2d(Rads: 0.09, Deg: 5.33))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.17), Rotation2d(Rads: 0.08, Deg: 4.84))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.09, Deg: 5.39))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.09, Deg: 5.21))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.10, Deg: 5.74))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.09, Deg: 5.25))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.09, Deg: 5.35))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.09, Deg: 5.23))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.08, Deg: 4.83))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.09, Deg: 4.88))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.09, Deg: 5.22))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.17), Rotation2d(Rads: 0.09, Deg: 5.12))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.17), Rotation2d(Rads: 0.08, Deg: 4.59))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.18), Rotation2d(Rads: 0.07, Deg: 4.21))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.18), Rotation2d(Rads: 0.07, Deg: 3.95))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.19), Rotation2d(Rads: 0.06, Deg: 3.46))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.19), Rotation2d(Rads: 0.05, Deg: 3.05))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.20), Rotation2d(Rads: 0.04, Deg: 2.19))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.20), Rotation2d(Rads: 0.04, Deg: 2.27))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.21), Rotation2d(Rads: 0.04, Deg: 2.02))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.21), Rotation2d(Rads: 0.03, Deg: 1.75))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.21), Rotation2d(Rads: 0.02, Deg: 1.36))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.21), Rotation2d(Rads: 0.03, Deg: 1.66))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.21), Rotation2d(Rads: 0.02, Deg: 1.37))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.21), Rotation2d(Rads: 0.02, Deg: 1.28))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.21), Rotation2d(Rads: 0.03, Deg: 1.63))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.21), Rotation2d(Rads: 0.01, Deg: 0.84))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.21), Rotation2d(Rads: 0.02, Deg: 0.95))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.21), Rotation2d(Rads: 0.02, Deg: 1.17))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.21), Rotation2d(Rads: 0.01, Deg: 0.80))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.21), Rotation2d(Rads: 0.02, Deg: 0.96))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.22), Rotation2d(Rads: 0.01, Deg: 0.38))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.22), Rotation2d(Rads: 0.01, Deg: 0.66))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.22), Rotation2d(Rads: 0.01, Deg: 0.51))
robot pose to target NWU Pose2d(Translation2d(X: -0.64, Y: -0.22), Rotation2d(Rads: 0.01, Deg: 0.42))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.22), Rotation2d(Rads: 0.00, Deg: 0.22))
robot pose to target NWU Pose2d(Translation2d(X: -0.64, Y: -0.22), Rotation2d(Rads: 0.01, Deg: 0.49))
robot pose to target NWU Pose2d(Translation2d(X: -0.64, Y: -0.23), Rotation2d(Rads: -0.00, Deg: -0.20))
robot pose to target NWU Pose2d(Translation2d(X: -0.64, Y: -0.23), Rotation2d(Rads: -0.01, Deg: -0.79))
robot pose to target NWU Pose2d(Translation2d(X: -0.64, Y: -0.23), Rotation2d(Rads: -0.01, Deg: -0.29))
robot pose to target NWU Pose2d(Translation2d(X: -0.64, Y: -0.23), Rotation2d(Rads: -0.02, Deg: -0.87))
robot pose to target NWU Pose2d(Translation2d(X: -0.64, Y: -0.23), Rotation2d(Rads: -0.02, Deg: -1.03))
robot pose to target NWU Pose2d(Translation2d(X: -0.64, Y: -0.23), Rotation2d(Rads: -0.01, Deg: -0.46))
robot pose to target NWU Pose2d(Translation2d(X: -0.64, Y: -0.23), Rotation2d(Rads: -0.01, Deg: -0.66))
robot pose to target NWU Pose2d(Translation2d(X: -0.64, Y: -0.22), Rotation2d(Rads: -0.00, Deg: -0.01))
robot pose to target NWU Pose2d(Translation2d(X: -0.64, Y: -0.22), Rotation2d(Rads: -0.00, Deg: -0.04))
robot pose to target NWU Pose2d(Translation2d(X: -0.64, Y: -0.22), Rotation2d(Rads: 0.01, Deg: 0.72))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.21), Rotation2d(Rads: -0.00, Deg: -0.11))
robot pose to target NWU Pose2d(Translation2d(X: -0.64, Y: -0.21), Rotation2d(Rads: 0.02, Deg: 1.41))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.20), Rotation2d(Rads: 0.01, Deg: 0.71))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.20), Rotation2d(Rads: 0.03, Deg: 1.91))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.20), Rotation2d(Rads: 0.01, Deg: 0.67))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.20), Rotation2d(Rads: 0.02, Deg: 0.88))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.20), Rotation2d(Rads: 0.02, Deg: 1.17))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.20), Rotation2d(Rads: 0.03, Deg: 1.99))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.20), Rotation2d(Rads: 0.03, Deg: 1.93))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.20), Rotation2d(Rads: 0.03, Deg: 1.60))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.18), Rotation2d(Rads: 0.05, Deg: 3.08))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.17), Rotation2d(Rads: 0.07, Deg: 3.89))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.17), Rotation2d(Rads: 0.06, Deg: 3.41))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.10, Deg: 5.46))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.09, Deg: 5.28))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.15), Rotation2d(Rads: 0.09, Deg: 5.05))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.10, Deg: 5.75))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.10, Deg: 5.64))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.10, Deg: 5.79))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.10, Deg: 5.96))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.09, Deg: 5.18))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.11, Deg: 6.03))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.09, Deg: 5.10))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.08, Deg: 4.82))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.10, Deg: 5.89))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.09, Deg: 5.11))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.09, Deg: 5.10))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.09, Deg: 4.94))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.09, Deg: 5.41))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.09, Deg: 5.17))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.10, Deg: 5.46))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.09, Deg: 5.04))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.09, Deg: 5.43))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.10, Deg: 5.62))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.09, Deg: 5.25))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.09, Deg: 5.05))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.09, Deg: 4.89))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.09, Deg: 5.42))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.09, Deg: 4.99))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.09, Deg: 5.00))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.08, Deg: 4.83))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.08, Deg: 4.74))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.08, Deg: 4.69))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.14), Rotation2d(Rads: 0.10, Deg: 5.92))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.08, Deg: 4.83))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.08, Deg: 4.87))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.09, Deg: 5.31))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.08, Deg: 4.60))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.08, Deg: 4.48))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.08, Deg: 4.47))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.07, Deg: 4.29))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.08, Deg: 4.60))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.08, Deg: 4.74))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.15), Rotation2d(Rads: 0.09, Deg: 5.15))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.09, Deg: 5.25))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.07, Deg: 4.14))
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.15), Rotation2d(Rads: 0.09, Deg: 5.30))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.09, Deg: 4.96))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.08, Deg: 4.63))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.08, Deg: 4.57))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.08, Deg: 4.81))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.09, Deg: 5.11))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.08, Deg: 4.83))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.09, Deg: 5.10))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.10, Deg: 5.70))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.08, Deg: 4.49))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.10, Deg: 5.47))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.08, Deg: 4.66))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.08, Deg: 4.69))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.08, Deg: 4.73))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.09, Deg: 5.09))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.08, Deg: 4.69))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.09, Deg: 5.14))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.08, Deg: 4.75))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.09, Deg: 5.10))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.09, Deg: 5.03))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.09, Deg: 5.19))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.10, Deg: 5.59))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.09, Deg: 5.16))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.10, Deg: 6.01))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.13), Rotation2d(Rads: 0.10, Deg: 5.46))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.13), Rotation2d(Rads: 0.10, Deg: 5.52))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.13), Rotation2d(Rads: 0.10, Deg: 5.98))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.13), Rotation2d(Rads: 0.10, Deg: 5.83))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.13), Rotation2d(Rads: 0.11, Deg: 6.24))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.13), Rotation2d(Rads: 0.10, Deg: 5.97))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.13), Rotation2d(Rads: 0.10, Deg: 5.90))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.13), Rotation2d(Rads: 0.11, Deg: 6.07))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.13), Rotation2d(Rads: 0.11, Deg: 6.07))
Align to reef command initialized
drive speeds 0.04508858109349237, 0.12283387494798695, 0.006145585407814134
drive speeds 0.04508858109349237, 0.12283387494798695, 0.006145585407814134
drive speeds 0.04508858109349237, 0.12283387494798695, 0.006145585407814134
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.13), Rotation2d(Rads: 0.10, Deg: 5.57))
drive speeds 0.04707689589635508, 0.12295571834340224, 0.005641684174199326
drive speeds 0.04707689589635508, 0.12295571834340224, 0.005641684174199326
drive speeds 0.04707689589635508, 0.12295571834340224, 0.005641684174199326
drive speeds 0.04707689589635508, 0.12295571834340224, 0.005641684174199326
drive speeds 0.04707689589635508, 0.12295571834340224, 0.005641684174199326
drive speeds 0.04707689589635508, 0.12295571834340224, 0.005641684174199326
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.13), Rotation2d(Rads: 0.10, Deg: 5.62))
drive speeds 0.049255743590374844, 0.121914055830978, 0.005693412289406905
drive speeds 0.049255743590374844, 0.121914055830978, 0.005693412289406905
drive speeds 0.049255743590374844, 0.121914055830978, 0.005693412289406905
drive speeds 0.049255743590374844, 0.121914055830978, 0.005693412289406905
drive speeds 0.049255743590374844, 0.121914055830978, 0.005693412289406905
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.13), Rotation2d(Rads: 0.10, Deg: 5.75))
drive speeds 0.04447672549074924, 0.12695980659212658, 0.005823703407331292
drive speeds 0.04447672549074924, 0.12695980659212658, 0.005823703407331292
drive speeds 0.04447672549074924, 0.12695980659212658, 0.005823703407331292
drive speeds 0.04447672549074924, 0.12695980659212658, 0.005823703407331292
drive speeds 0.04447672549074924, 0.12695980659212658, 0.005823703407331292
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.13), Rotation2d(Rads: 0.11, Deg: 6.25))
drive speeds 0.04026981012477858, 0.134672007802374, 0.006324146478331202
drive speeds 0.04026981012477858, 0.134672007802374, 0.006324146478331202
drive speeds 0.04026981012477858, 0.134672007802374, 0.006324146478331202
drive speeds 0.04026981012477858, 0.134672007802374, 0.006324146478331202
drive speeds 0.04026981012477858, 0.134672007802374, 0.006324146478331202
drive speeds 0.04026981012477858, 0.134672007802374, 0.006324146478331202
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.13), Rotation2d(Rads: 0.10, Deg: 5.90))
drive speeds 0.04508829310332659, 0.1317305699228644, 0.00597122545841025
drive speeds 0.04508829310332659, 0.1317305699228644, 0.00597122545841025
drive speeds 0.04508829310332659, 0.1317305699228644, 0.00597122545841025
drive speeds 0.04508829310332659, 0.1317305699228644, 0.00597122545841025
drive speeds 0.04508829310332659, 0.1317305699228644, 0.00597122545841025
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.13), Rotation2d(Rads: 0.10, Deg: 5.91))
drive speeds 0.04654116753190113, 0.1305657918209848, 0.005981758926448135
drive speeds 0.04654116753190113, 0.1305657918209848, 0.005981758926448135
drive speeds 0.04654116753190113, 0.1305657918209848, 0.005981758926448135
drive speeds 0.04654116753190113, 0.1305657918209848, 0.005981758926448135
drive speeds 0.04654116753190113, 0.1305657918209848, 0.005981758926448135
drive speeds 0.04654116753190113, 0.1305657918209848, 0.005981758926448135
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.13), Rotation2d(Rads: 0.10, Deg: 5.75))
drive speeds 0.04554752349924807, 0.13724440406742716, 0.005816063387198739
drive speeds 0.04554752349924807, 0.13724440406742716, 0.005816063387198739
drive speeds 0.04554752349924807, 0.13724440406742716, 0.005816063387198739
drive speeds 0.04554752349924807, 0.13724440406742716, 0.005816063387198739
drive speeds 0.04554752349924807, 0.13724440406742716, 0.005816063387198739
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.12), Rotation2d(Rads: 0.11, Deg: 6.22))
drive speeds 0.04424395737490776, 0.15656644130386566, 0.006295576704824379
drive speeds 0.04424395737490776, 0.15656644130386566, 0.006295576704824379
drive speeds 0.04424395737490776, 0.15656644130386566, 0.006295576704824379
drive speeds 0.04424395737490776, 0.15656644130386566, 0.006295576704824379
drive speeds 0.04424395737490776, 0.15656644130386566, 0.006295576704824379
drive speeds 0.04424395737490776, 0.15656644130386566, 0.006295576704824379
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.12), Rotation2d(Rads: 0.12, Deg: 6.83))
drive speeds 0.039310691031139045, 0.16677087548123393, 0.006918204298202062
drive speeds 0.039310691031139045, 0.16677087548123393, 0.006918204298202062
drive speeds 0.039310691031139045, 0.16677087548123393, 0.006918204298202062
drive speeds 0.039310691031139045, 0.16677087548123393, 0.006918204298202062
DataLog: Renamed log file from 'FRC_TBD_6f37cd4940e00a92.wpilog' to 'FRC_20250401_015536.wpilog'
drive speeds 0.039310691031139045, 0.16677087548123393, 0.006918204298202062
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.12), Rotation2d(Rads: 0.12, Deg: 6.85))
drive speeds 0.045278863271582315, 0.1587456645868304, 0.006937638002683833
drive speeds 0.045278863271582315, 0.1587456645868304, 0.006937638002683833
drive speeds 0.045278863271582315, 0.1587456645868304, 0.006937638002683833
drive speeds 0.045278863271582315, 0.1587456645868304, 0.006937638002683833
drive speeds 0.045278863271582315, 0.1587456645868304, 0.006937638002683833
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.12), Rotation2d(Rads: 0.12, Deg: 7.09))
drive speeds 0.04852229248724525, 0.1602221448670885, 0.007174643351020094
drive speeds 0.04852229248724525, 0.1602221448670885, 0.007174643351020094
drive speeds 0.04852229248724525, 0.1602221448670885, 0.007174643351020094
drive speeds 0.04852229248724525, 0.1602221448670885, 0.007174643351020094
drive speeds 0.04852229248724525, 0.1602221448670885, 0.007174643351020094
drive speeds 0.04852229248724525, 0.1602221448670885, 0.007174643351020094
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.13), Rotation2d(Rads: 0.11, Deg: 6.26))
drive speeds 0.05515750645898355, 0.14547212093348705, 0.006341242838422962
drive speeds 0.05515750645898355, 0.14547212093348705, 0.006341242838422962
drive speeds 0.05515750645898355, 0.14547212093348705, 0.006341242838422962
drive speeds 0.05515750645898355, 0.14547212093348705, 0.006341242838422962
drive speeds 0.05515750645898355, 0.14547212093348705, 0.006341242838422962
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.13), Rotation2d(Rads: 0.10, Deg: 5.96))
drive speeds 0.05909012218951219, 0.1369467889726465, 0.006037793246538093
drive speeds 0.05909012218951219, 0.1369467889726465, 0.006037793246538093
drive speeds 0.05909012218951219, 0.1369467889726465, 0.006037793246538093
drive speeds 0.05909012218951219, 0.1369467889726465, 0.006037793246538093
drive speeds 0.05909012218951219, 0.1369467889726465, 0.006037793246538093
drive speeds 0.05909012218951219, 0.1369467889726465, 0.006037793246538093
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.13), Rotation2d(Rads: 0.10, Deg: 5.96))
drive speeds 0.0578551566449402, 0.13665023529945142, 0.006031512570003956
drive speeds 0.0578551566449402, 0.13665023529945142, 0.006031512570003956
drive speeds 0.0578551566449402, 0.13665023529945142, 0.006031512570003956
drive speeds 0.0578551566449402, 0.13665023529945142, 0.006031512570003956
drive speeds 0.0578551566449402, 0.13665023529945142, 0.006031512570003956
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.13), Rotation2d(Rads: 0.11, Deg: 6.57))
drive speeds 0.05399146011430947, 0.1430042708424238, 0.006651457173390316
drive speeds 0.05399146011430947, 0.1430042708424238, 0.006651457173390316
drive speeds 0.05399146011430947, 0.1430042708424238, 0.006651457173390316
drive speeds 0.05399146011430947, 0.1430042708424238, 0.006651457173390316
drive speeds 0.05399146011430947, 0.1430042708424238, 0.006651457173390316
drive speeds 0.05399146011430947, 0.1430042708424238, 0.006651457173390316
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.12), Rotation2d(Rads: 0.12, Deg: 6.83))
drive speeds 0.05126400315060415, 0.15175394256707603, 0.006917756187254113
drive speeds 0.05126400315060415, 0.15175394256707603, 0.006917756187254113
drive speeds 0.05126400315060415, 0.15175394256707603, 0.006917756187254113
drive speeds 0.05126400315060415, 0.15175394256707603, 0.006917756187254113
drive speeds 0.05126400315060415, 0.15175394256707603, 0.006917756187254113
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.12), Rotation2d(Rads: 0.12, Deg: 6.82))
drive speeds 0.05421703815693012, 0.15038658472192407, 0.006905297466702208
drive speeds 0.05421703815693012, 0.15038658472192407, 0.006905297466702208
drive speeds 0.05421703815693012, 0.15038658472192407, 0.006905297466702208
drive speeds 0.05421703815693012, 0.15038658472192407, 0.006905297466702208
drive speeds 0.05421703815693012, 0.15038658472192407, 0.006905297466702208
drive speeds 0.05421703815693012, 0.15038658472192407, 0.006905297466702208
drive speeds 0.05421703815693012, 0.15038658472192407, 0.006905297466702208
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.13), Rotation2d(Rads: 0.11, Deg: 6.11))
drive speeds 0.0566312970745782, 0.14252523669989633, 0.006190018132395079
drive speeds 0.0566312970745782, 0.14252523669989633, 0.006190018132395079
drive speeds 0.0566312970745782, 0.14252523669989633, 0.006190018132395079
drive speeds 0.0566312970745782, 0.14252523669989633, 0.006190018132395079
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.12), Rotation2d(Rads: 0.12, Deg: 6.79))
drive speeds 0.05222758314124038, 0.14987263988460542, 0.0068781443091924304
drive speeds 0.05222758314124038, 0.14987263988460542, 0.0068781443091924304
drive speeds 0.05222758314124038, 0.14987263988460542, 0.0068781443091924304
drive speeds 0.05222758314124038, 0.14987263988460542, 0.0068781443091924304
drive speeds 0.05222758314124038, 0.14987263988460542, 0.0068781443091924304
drive speeds 0.05222758314124038, 0.14987263988460542, 0.0068781443091924304
drive speeds 0.05222758314124038, 0.14987263988460542, 0.0068781443091924304
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.12), Rotation2d(Rads: 0.11, Deg: 6.31))
drive speeds 0.05306196560784357, 0.15055009053898774, 0.006384495048855218
drive speeds 0.05306196560784357, 0.15055009053898774, 0.006384495048855218
drive speeds 0.05306196560784357, 0.15055009053898774, 0.006384495048855218
drive speeds 0.05306196560784357, 0.15055009053898774, 0.006384495048855218
drive speeds 0.05306196560784357, 0.15055009053898774, 0.006384495048855218
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.12), Rotation2d(Rads: 0.11, Deg: 6.05))
drive speeds 0.055406415066235476, 0.16216656148280148, 0.006120174845415314
drive speeds 0.055406415066235476, 0.16216656148280148, 0.006120174845415314
drive speeds 0.055406415066235476, 0.16216656148280148, 0.006120174845415314
drive speeds 0.055406415066235476, 0.16216656148280148, 0.006120174845415314
drive speeds 0.055406415066235476, 0.16216656148280148, 0.006120174845415314
drive speeds 0.055406415066235476, 0.16216656148280148, 0.006120174845415314
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.12), Rotation2d(Rads: 0.12, Deg: 6.81))
drive speeds 0.04923451044059855, 0.1665545226104043, 0.006889309177577431
drive speeds 0.04923451044059855, 0.1665545226104043, 0.006889309177577431
drive speeds 0.04923451044059855, 0.1665545226104043, 0.006889309177577431
drive speeds 0.04923451044059855, 0.1665545226104043, 0.006889309177577431
drive speeds 0.04923451044059855, 0.1665545226104043, 0.006889309177577431
drive speeds 0.04923451044059855, 0.1665545226104043, 0.006889309177577431
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.11), Rotation2d(Rads: 0.13, Deg: 7.19))
drive speeds 0.05935439760868749, 0.1847625715179969, 0.007273988495210649
drive speeds 0.05935439760868749, 0.1847625715179969, 0.007273988495210649
drive speeds 0.05935439760868749, 0.1847625715179969, 0.007273988495210649
drive speeds 0.05935439760868749, 0.1847625715179969, 0.007273988495210649
drive speeds 0.05935439760868749, 0.1847625715179969, 0.007273988495210649
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.12), Rotation2d(Rads: 0.12, Deg: 6.90))
drive speeds 0.06151438730629931, 0.1793561329570338, 0.006987027848928464
drive speeds 0.06151438730629931, 0.1793561329570338, 0.006987027848928464
drive speeds 0.06151438730629931, 0.1793561329570338, 0.006987027848928464
drive speeds 0.06151438730629931, 0.1793561329570338, 0.006987027848928464
drive speeds 0.06151438730629931, 0.1793561329570338, 0.006987027848928464
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.11), Rotation2d(Rads: 0.12, Deg: 6.74))
drive speeds 0.0581916724448417, 0.1818226139815192, 0.006826438025718389
drive speeds 0.0581916724448417, 0.1818226139815192, 0.006826438025718389
drive speeds 0.0581916724448417, 0.1818226139815192, 0.006826438025718389
drive speeds 0.0581916724448417, 0.1818226139815192, 0.006826438025718389
drive speeds 0.0581916724448417, 0.1818226139815192, 0.006826438025718389
drive speeds 0.0581916724448417, 0.1818226139815192, 0.006826438025718389
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.11), Rotation2d(Rads: 0.12, Deg: 6.77))
drive speeds 0.06015628013936891, 0.18717167126756284, 0.006851151052399535
drive speeds 0.06015628013936891, 0.18717167126756284, 0.006851151052399535
drive speeds 0.06015628013936891, 0.18717167126756284, 0.006851151052399535
drive speeds 0.06015628013936891, 0.18717167126756284, 0.006851151052399535
drive speeds 0.06015628013936891, 0.18717167126756284, 0.006851151052399535
drive speeds 0.06015628013936891, 0.18717167126756284, 0.006851151052399535
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.11), Rotation2d(Rads: 0.12, Deg: 6.90))
drive speeds 0.05565334640724953, 0.19679580837311741, 0.006988600651351916
drive speeds 0.05565334640724953, 0.19679580837311741, 0.006988600651351916
drive speeds 0.05565334640724953, 0.19679580837311741, 0.006988600651351916
drive speeds 0.05565334640724953, 0.19679580837311741, 0.006988600651351916
drive speeds 0.05565334640724953, 0.19679580837311741, 0.006988600651351916
drive speeds 0.05565334640724953, 0.19679580837311741, 0.006988600651351916
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.11), Rotation2d(Rads: 0.13, Deg: 7.29))
drive speeds 0.05280327057052611, 0.20475109616776363, 0.007383106452192093
drive speeds 0.05280327057052611, 0.20475109616776363, 0.007383106452192093
drive speeds 0.05280327057052611, 0.20475109616776363, 0.007383106452192093
drive speeds 0.05280327057052611, 0.20475109616776363, 0.007383106452192093
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.11), Rotation2d(Rads: 0.12, Deg: 7.09))
drive speeds 0.051735836290143195, 0.20076511619394563, 0.007173788211854558
drive speeds 0.051735836290143195, 0.20076511619394563, 0.007173788211854558
drive speeds 0.051735836290143195, 0.20076511619394563, 0.007173788211854558
drive speeds 0.051735836290143195, 0.20076511619394563, 0.007173788211854558
drive speeds 0.051735836290143195, 0.20076511619394563, 0.007173788211854558
drive speeds 0.051735836290143195, 0.20076511619394563, 0.007173788211854558
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.11), Rotation2d(Rads: 0.13, Deg: 7.43))
drive speeds 0.049411583392187486, 0.2115666789847696, 0.00751909626400643
drive speeds 0.049411583392187486, 0.2115666789847696, 0.00751909626400643
drive speeds 0.049411583392187486, 0.2115666789847696, 0.00751909626400643
drive speeds 0.049411583392187486, 0.2115666789847696, 0.00751909626400643
drive speeds 0.049411583392187486, 0.2115666789847696, 0.00751909626400643
drive speeds 0.049411583392187486, 0.2115666789847696, 0.00751909626400643
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.10), Rotation2d(Rads: 0.13, Deg: 7.60))
drive speeds 0.05113440740104259, 0.22493066525721284, 0.007692131261735876
drive speeds 0.05113440740104259, 0.22493066525721284, 0.007692131261735876
drive speeds 0.05113440740104259, 0.22493066525721284, 0.007692131261735876
drive speeds 0.05113440740104259, 0.22493066525721284, 0.007692131261735876
drive speeds 0.05113440740104259, 0.22493066525721284, 0.007692131261735876
drive speeds 0.05113440740104259, 0.22493066525721284, 0.007692131261735876
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.10), Rotation2d(Rads: 0.12, Deg: 7.05))
drive speeds 0.05140649321336452, 0.23149380394175234, 0.007139945286908729
drive speeds 0.05140649321336452, 0.23149380394175234, 0.007139945286908729
drive speeds 0.05140649321336452, 0.23149380394175234, 0.007139945286908729
drive speeds 0.05140649321336452, 0.23149380394175234, 0.007139945286908729
drive speeds 0.05140649321336452, 0.23149380394175234, 0.007139945286908729
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.10), Rotation2d(Rads: 0.13, Deg: 7.39))
drive speeds 0.04537818030409374, 0.23661017745610105, 0.007485773666603209
drive speeds 0.04537818030409374, 0.23661017745610105, 0.007485773666603209
drive speeds 0.04537818030409374, 0.23661017745610105, 0.007485773666603209
drive speeds 0.04537818030409374, 0.23661017745610105, 0.007485773666603209
drive speeds 0.04537818030409374, 0.23661017745610105, 0.007485773666603209
drive speeds 0.04537818030409374, 0.23661017745610105, 0.007485773666603209
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.10), Rotation2d(Rads: 0.13, Deg: 7.54))
drive speeds 0.043608773682209306, 0.23714547173624392, 0.007635346355804941
drive speeds 0.043608773682209306, 0.23714547173624392, 0.007635346355804941
drive speeds 0.043608773682209306, 0.23714547173624392, 0.007635346355804941
drive speeds 0.043608773682209306, 0.23714547173624392, 0.007635346355804941
drive speeds 0.043608773682209306, 0.23714547173624392, 0.007635346355804941
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.10), Rotation2d(Rads: 0.13, Deg: 7.53))
drive speeds 0.04130530772848534, 0.22559984539046132, 0.007618267953293002
drive speeds 0.04130530772848534, 0.22559984539046132, 0.007618267953293002
drive speeds 0.04130530772848534, 0.22559984539046132, 0.007618267953293002
drive speeds 0.04130530772848534, 0.22559984539046132, 0.007618267953293002
drive speeds 0.04130530772848534, 0.22559984539046132, 0.007618267953293002
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.11), Rotation2d(Rads: 0.12, Deg: 7.05))
drive speeds 0.04627372304047654, 0.21439895375288373, 0.0071404097628889095
drive speeds 0.04627372304047654, 0.21439895375288373, 0.0071404097628889095
drive speeds 0.04627372304047654, 0.21439895375288373, 0.0071404097628889095
drive speeds 0.04627372304047654, 0.21439895375288373, 0.0071404097628889095
drive speeds 0.04627372304047654, 0.21439895375288373, 0.0071404097628889095
drive speeds 0.04627372304047654, 0.21439895375288373, 0.0071404097628889095
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.11), Rotation2d(Rads: 0.13, Deg: 7.19))
drive speeds 0.04598818935841553, 0.1987809931030951, 0.007279274315478915
drive speeds 0.04598818935841553, 0.1987809931030951, 0.007279274315478915
drive speeds 0.04598818935841553, 0.1987809931030951, 0.007279274315478915
drive speeds 0.04598818935841553, 0.1987809931030951, 0.007279274315478915
drive speeds 0.04598818935841553, 0.1987809931030951, 0.007279274315478915
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.11), Rotation2d(Rads: 0.12, Deg: 6.94))
drive speeds 0.0453733408376348, 0.18921682651958405, 0.0070202626339682505
drive speeds 0.0453733408376348, 0.18921682651958405, 0.0070202626339682505
drive speeds 0.0453733408376348, 0.18921682651958405, 0.0070202626339682505
drive speeds 0.0453733408376348, 0.18921682651958405, 0.0070202626339682505
drive speeds 0.0453733408376348, 0.18921682651958405, 0.0070202626339682505
drive speeds 0.0453733408376348, 0.18921682651958405, 0.0070202626339682505
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.12), Rotation2d(Rads: 0.12, Deg: 6.76))
drive speeds 0.037433886207123145, 0.17968203330136648, 0.006838626524338037
drive speeds 0.037433886207123145, 0.17968203330136648, 0.006838626524338037
drive speeds 0.037433886207123145, 0.17968203330136648, 0.006838626524338037
drive speeds 0.037433886207123145, 0.17968203330136648, 0.006838626524338037
drive speeds 0.037433886207123145, 0.17968203330136648, 0.006838626524338037
drive speeds 0.037433886207123145, 0.17968203330136648, 0.006838626524338037
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.12), Rotation2d(Rads: 0.11, Deg: 6.07))
drive speeds 0.03817655605213244, 0.18066673563698193, 0.006140648789824354
drive speeds 0.03817655605213244, 0.18066673563698193, 0.006140648789824354
drive speeds 0.03817655605213244, 0.18066673563698193, 0.006140648789824354
drive speeds 0.03817655605213244, 0.18066673563698193, 0.006140648789824354
drive speeds 0.03817655605213244, 0.18066673563698193, 0.006140648789824354
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.11), Rotation2d(Rads: 0.11, Deg: 6.25))
drive speeds 0.040991541337324154, 0.18839470388757693, 0.006328369372150882
drive speeds 0.040991541337324154, 0.18839470388757693, 0.006328369372150882
drive speeds 0.040991541337324154, 0.18839470388757693, 0.006328369372150882
drive speeds 0.040991541337324154, 0.18839470388757693, 0.006328369372150882
drive speeds 0.040991541337324154, 0.18839470388757693, 0.006328369372150882
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.11), Rotation2d(Rads: 0.12, Deg: 6.82))
drive speeds 0.036857231757820616, 0.1933422972025684, 0.006902352748016104
drive speeds 0.036857231757820616, 0.1933422972025684, 0.006902352748016104
drive speeds 0.036857231757820616, 0.1933422972025684, 0.006902352748016104
drive speeds 0.036857231757820616, 0.1933422972025684, 0.006902352748016104
drive speeds 0.036857231757820616, 0.1933422972025684, 0.006902352748016104
drive speeds 0.036857231757820616, 0.1933422972025684, 0.006902352748016104
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.11), Rotation2d(Rads: 0.12, Deg: 6.85))
drive speeds 0.03283997124544917, 0.19664695201578608, 0.006934779790076238
drive speeds 0.03283997124544917, 0.19664695201578608, 0.006934779790076238
drive speeds 0.03283997124544917, 0.19664695201578608, 0.006934779790076238
drive speeds 0.03283997124544917, 0.19664695201578608, 0.006934779790076238
drive speeds 0.03283997124544917, 0.19664695201578608, 0.006934779790076238
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.11), Rotation2d(Rads: 0.11, Deg: 6.48))
drive speeds 0.03344268222211243, 0.1968933272098264, 0.0065593323942703795
drive speeds 0.03344268222211243, 0.1968933272098264, 0.0065593323942703795
drive speeds 0.03344268222211243, 0.1968933272098264, 0.0065593323942703795
drive speeds 0.03344268222211243, 0.1968933272098264, 0.0065593323942703795
drive speeds 0.03344268222211243, 0.1968933272098264, 0.0065593323942703795
drive speeds 0.03344268222211243, 0.1968933272098264, 0.0065593323942703795
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.11), Rotation2d(Rads: 0.12, Deg: 6.78))
drive speeds 0.03215045148225047, 0.1978769978089739, 0.006864780559829951
drive speeds 0.03215045148225047, 0.1978769978089739, 0.006864780559829951
drive speeds 0.03215045148225047, 0.1978769978089739, 0.006864780559829951
drive speeds 0.03215045148225047, 0.1978769978089739, 0.006864780559829951
drive speeds 0.03215045148225047, 0.1978769978089739, 0.006864780559829951
drive speeds 0.03215045148225047, 0.1978769978089739, 0.006864780559829951
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.11), Rotation2d(Rads: 0.11, Deg: 6.12))
drive speeds 0.035036569962128936, 0.19945297663733186, 0.006198337303400589
drive speeds 0.035036569962128936, 0.19945297663733186, 0.006198337303400589
drive speeds 0.035036569962128936, 0.19945297663733186, 0.006198337303400589
drive speeds 0.035036569962128936, 0.19945297663733186, 0.006198337303400589
drive speeds 0.035036569962128936, 0.19945297663733186, 0.006198337303400589
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.11), Rotation2d(Rads: 0.12, Deg: 6.82))
drive speeds 0.027736617655992866, 0.2005894989929178, 0.006901924251557018
drive speeds 0.027736617655992866, 0.2005894989929178, 0.006901924251557018
drive speeds 0.027736617655992866, 0.2005894989929178, 0.006901924251557018
drive speeds 0.027736617655992866, 0.2005894989929178, 0.006901924251557018
drive speeds 0.027736617655992866, 0.2005894989929178, 0.006901924251557018
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.11), Rotation2d(Rads: 0.11, Deg: 6.04))
drive speeds 0.036022520780573906, 0.2097081204544485, 0.006115408698165551
drive speeds 0.036022520780573906, 0.2097081204544485, 0.006115408698165551
drive speeds 0.036022520780573906, 0.2097081204544485, 0.006115408698165551
drive speeds 0.036022520780573906, 0.2097081204544485, 0.006115408698165551
drive speeds 0.036022520780573906, 0.2097081204544485, 0.006115408698165551
drive speeds 0.036022520780573906, 0.2097081204544485, 0.006115408698165551
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.10), Rotation2d(Rads: 0.12, Deg: 6.74))
drive speeds 0.02934400123144587, 0.21700299264691383, 0.006819789997493316
drive speeds 0.02934400123144587, 0.21700299264691383, 0.006819789997493316
drive speeds 0.02934400123144587, 0.21700299264691383, 0.006819789997493316
drive speeds 0.02934400123144587, 0.21700299264691383, 0.006819789997493316
drive speeds 0.02934400123144587, 0.21700299264691383, 0.006819789997493316
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.10), Rotation2d(Rads: 0.12, Deg: 6.74))
drive speeds 0.03658889682255547, 0.21357338704038714, 0.006303581109782418
drive speeds 0.03658889682255547, 0.21357338704038714, 0.006303581109782418
drive speeds 0.03658889682255547, 0.21357338704038714, 0.006303581109782418
drive speeds 0.03658889682255547, 0.21357338704038714, 0.006303581109782418
drive speeds 0.03658889682255547, 0.21357338704038714, 0.006303581109782418
drive speeds 0.03658889682255547, 0.21357338704038714, 0.006303581109782418
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.11), Rotation2d(Rads: 0.11, Deg: 6.51))
drive speeds 0.03216226294192257, 0.2090453581458196, 0.006587095216028477
drive speeds 0.03216226294192257, 0.2090453581458196, 0.006587095216028477
drive speeds 0.03216226294192257, 0.2090453581458196, 0.006587095216028477
drive speeds 0.03216226294192257, 0.2090453581458196, 0.006587095216028477
drive speeds 0.03216226294192257, 0.2090453581458196, 0.006587095216028477
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.10), Rotation2d(Rads: 0.11, Deg: 6.40))
drive speeds 0.027452125519146064, 0.22244409087277647, 0.006478062439301533
drive speeds 0.027452125519146064, 0.22244409087277647, 0.006478062439301533
drive speeds 0.027452125519146064, 0.22244409087277647, 0.006478062439301533
drive speeds 0.027452125519146064, 0.22244409087277647, 0.006478062439301533
drive speeds 0.027452125519146064, 0.22244409087277647, 0.006478062439301533
drive speeds 0.027452125519146064, 0.22244409087277647, 0.006478062439301533
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.10), Rotation2d(Rads: 0.12, Deg: 7.11))
drive speeds 0.021218283422572957, 0.22559652312485778, 0.007193710997686103
drive speeds 0.021218283422572957, 0.22559652312485778, 0.007193710997686103
drive speeds 0.021218283422572957, 0.22559652312485778, 0.007193710997686103
drive speeds 0.021218283422572957, 0.22559652312485778, 0.007193710997686103
drive speeds 0.021218283422572957, 0.22559652312485778, 0.007193710997686103
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.10), Rotation2d(Rads: 0.12, Deg: 6.71))
drive speeds 0.021706492795215714, 0.2300793253037555, 0.006793717005352991
drive speeds 0.021706492795215714, 0.2300793253037555, 0.006793717005352991
drive speeds 0.021706492795215714, 0.2300793253037555, 0.006793717005352991
drive speeds 0.021706492795215714, 0.2300793253037555, 0.006793717005352991
drive speeds 0.021706492795215714, 0.2300793253037555, 0.006793717005352991
drive speeds 0.021706492795215714, 0.2300793253037555, 0.006793717005352991
drive speeds 0.021706492795215714, 0.2300793253037555, 0.006793717005352991
drive speeds 0.021706492795215714, 0.2300793253037555, 0.006793717005352991
drive speeds 0.021706492795215714, 0.2300793253037555, 0.006793717005352991
drive speeds 0.021706492795215714, 0.2300793253037555, 0.006793717005352991
drive speeds 0.021706492795215714, 0.2300793253037555, 0.006793717005352991
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.10), Rotation2d(Rads: 0.11, Deg: 6.24))
drive speeds 0.02293481027709525, 0.22141837104615758, 0.006311992762400978
drive speeds 0.02293481027709525, 0.22141837104615758, 0.006311992762400978
drive speeds 0.02293481027709525, 0.22141837104615758, 0.006311992762400978
drive speeds 0.02293481027709525, 0.22141837104615758, 0.006311992762400978
drive speeds 0.02293481027709525, 0.22141837104615758, 0.006311992762400978
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.10), Rotation2d(Rads: 0.12, Deg: 6.97))
drive speeds 0.026348433833113714, 0.23910186796944025, 0.007053070257439228
drive speeds 0.026348433833113714, 0.23910186796944025, 0.007053070257439228
drive speeds 0.026348433833113714, 0.23910186796944025, 0.007053070257439228
drive speeds 0.026348433833113714, 0.23910186796944025, 0.007053070257439228
drive speeds 0.026348433833113714, 0.23910186796944025, 0.007053070257439228
drive speeds 0.026348433833113714, 0.23910186796944025, 0.007053070257439228
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.09), Rotation2d(Rads: 0.12, Deg: 6.95))
drive speeds 0.021622988594537294, 0.2550340740264402, 0.00703593067228589
drive speeds 0.021622988594537294, 0.2550340740264402, 0.00703593067228589
drive speeds 0.021622988594537294, 0.2550340740264402, 0.00703593067228589
drive speeds 0.021622988594537294, 0.2550340740264402, 0.00703593067228589
drive speeds 0.021622988594537294, 0.2550340740264402, 0.00703593067228589
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.09), Rotation2d(Rads: 0.12, Deg: 6.88))
drive speeds 0.01584448384630813, 0.26321294143742807, 0.006966437186797915
drive speeds 0.01584448384630813, 0.26321294143742807, 0.006966437186797915
drive speeds 0.01584448384630813, 0.26321294143742807, 0.006966437186797915
drive speeds 0.01584448384630813, 0.26321294143742807, 0.006966437186797915
drive speeds 0.01584448384630813, 0.26321294143742807, 0.006966437186797915
drive speeds 0.01584448384630813, 0.26321294143742807, 0.006966437186797915
drive speeds 0.01584448384630813, 0.26321294143742807, 0.006966437186797915
drive speeds 0.01584448384630813, 0.26321294143742807, 0.006966437186797915
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.09), Rotation2d(Rads: 0.13, Deg: 7.19))
drive speeds 0.015271010043687472, 0.262465141669144, 0.007278033732942847
drive speeds 0.015271010043687472, 0.262465141669144, 0.007278033732942847
drive speeds 0.015271010043687472, 0.262465141669144, 0.007278033732942847
drive speeds 0.015271010043687472, 0.262465141669144, 0.007278033732942847
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.09), Rotation2d(Rads: 0.13, Deg: 7.30))
drive speeds 0.016499661844115387, 0.27031195473214725, 0.007385761303113787
drive speeds 0.016499661844115387, 0.27031195473214725, 0.007385761303113787
drive speeds 0.016499661844115387, 0.27031195473214725, 0.007385761303113787
drive speeds 0.016499661844115387, 0.27031195473214725, 0.007385761303113787
drive speeds 0.016499661844115387, 0.27031195473214725, 0.007385761303113787
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.09), Rotation2d(Rads: 0.13, Deg: 7.36))
drive speeds 0.02403806632598634, 0.26403107578432344, 0.007448237362593126
drive speeds 0.02403806632598634, 0.26403107578432344, 0.007448237362593126
drive speeds 0.02403806632598634, 0.26403107578432344, 0.007448237362593126
drive speeds 0.02403806632598634, 0.26403107578432344, 0.007448237362593126
drive speeds 0.02403806632598634, 0.26403107578432344, 0.007448237362593126
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.10), Rotation2d(Rads: 0.11, Deg: 6.43))
drive speeds 0.028195456182784937, 0.23893453675098855, 0.006513084157927363
drive speeds 0.028195456182784937, 0.23893453675098855, 0.006513084157927363
drive speeds 0.028195456182784937, 0.23893453675098855, 0.006513084157927363
drive speeds 0.028195456182784937, 0.23893453675098855, 0.006513084157927363
drive speeds 0.028195456182784937, 0.23893453675098855, 0.006513084157927363
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.10), Rotation2d(Rads: 0.12, Deg: 7.01))
drive speeds 0.028370973851431235, 0.2375222354007004, 0.007094914089610071
drive speeds 0.028370973851431235, 0.2375222354007004, 0.007094914089610071
drive speeds 0.028370973851431235, 0.2375222354007004, 0.007094914089610071
drive speeds 0.028370973851431235, 0.2375222354007004, 0.007094914089610071
drive speeds 0.028370973851431235, 0.2375222354007004, 0.007094914089610071
drive speeds 0.028370973851431235, 0.2375222354007004, 0.007094914089610071
drive speeds 0.028370973851431235, 0.2375222354007004, 0.007094914089610071
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.10), Rotation2d(Rads: 0.12, Deg: 6.60))
drive speeds 0.035894767206285205, 0.23775017873155507, 0.006676336536986138
drive speeds 0.035894767206285205, 0.23775017873155507, 0.006676336536986138
drive speeds 0.035894767206285205, 0.23775017873155507, 0.006676336536986138
drive speeds 0.035894767206285205, 0.23775017873155507, 0.006676336536986138
drive speeds 0.035894767206285205, 0.23775017873155507, 0.006676336536986138
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.09), Rotation2d(Rads: 0.13, Deg: 7.39))
drive speeds 0.0379940615857533, 0.2482453254468794, 0.007481001150808936
drive speeds 0.0379940615857533, 0.2482453254468794, 0.007481001150808936
drive speeds 0.0379940615857533, 0.2482453254468794, 0.007481001150808936
drive speeds 0.0379940615857533, 0.2482453254468794, 0.007481001150808936
drive speeds 0.0379940615857533, 0.2482453254468794, 0.007481001150808936
drive speeds 0.0379940615857533, 0.2482453254468794, 0.007481001150808936
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.09), Rotation2d(Rads: 0.14, Deg: 7.78))
drive speeds 0.029112621446961227, 0.259446864510244, 0.007880366614517246
drive speeds 0.029112621446961227, 0.259446864510244, 0.007880366614517246
drive speeds 0.029112621446961227, 0.259446864510244, 0.007880366614517246
drive speeds 0.029112621446961227, 0.259446864510244, 0.007880366614517246
drive speeds 0.029112621446961227, 0.259446864510244, 0.007880366614517246
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.09), Rotation2d(Rads: 0.13, Deg: 7.48))
drive speeds 0.030167234838437716, 0.26744954276813143, 0.007569315216917294
drive speeds 0.030167234838437716, 0.26744954276813143, 0.007569315216917294
drive speeds 0.030167234838437716, 0.26744954276813143, 0.007569315216917294
drive speeds 0.030167234838437716, 0.26744954276813143, 0.007569315216917294
drive speeds 0.030167234838437716, 0.26744954276813143, 0.007569315216917294
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.09), Rotation2d(Rads: 0.13, Deg: 7.64))
drive speeds 0.025154927619203404, 0.2718596931181757, 0.007732379694899665
drive speeds 0.025154927619203404, 0.2718596931181757, 0.007732379694899665
drive speeds 0.025154927619203404, 0.2718596931181757, 0.007732379694899665
drive speeds 0.025154927619203404, 0.2718596931181757, 0.007732379694899665
drive speeds 0.025154927619203404, 0.2718596931181757, 0.007732379694899665
drive speeds 0.025154927619203404, 0.2718596931181757, 0.007732379694899665
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.09), Rotation2d(Rads: 0.14, Deg: 7.76))
drive speeds 0.032000268483864815, 0.2730144440090777, 0.007859800985099129
drive speeds 0.032000268483864815, 0.2730144440090777, 0.007859800985099129
drive speeds 0.032000268483864815, 0.2730144440090777, 0.007859800985099129
drive speeds 0.032000268483864815, 0.2730144440090777, 0.007859800985099129
drive speeds 0.032000268483864815, 0.2730144440090777, 0.007859800985099129
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.09), Rotation2d(Rads: 0.13, Deg: 7.55))
drive speeds 0.029421439197769315, 0.27206744779695835, 0.007640619826169738
drive speeds 0.029421439197769315, 0.27206744779695835, 0.007640619826169738
drive speeds 0.029421439197769315, 0.27206744779695835, 0.007640619826169738
drive speeds 0.029421439197769315, 0.27206744779695835, 0.007640619826169738
drive speeds 0.029421439197769315, 0.27206744779695835, 0.007640619826169738
drive speeds 0.029421439197769315, 0.27206744779695835, 0.007640619826169738
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.09), Rotation2d(Rads: 0.14, Deg: 7.96))
drive speeds 0.030346733225820686, 0.2672118848982901, 0.008062834854479485
drive speeds 0.030346733225820686, 0.2672118848982901, 0.008062834854479485
drive speeds 0.030346733225820686, 0.2672118848982901, 0.008062834854479485
drive speeds 0.030346733225820686, 0.2672118848982901, 0.008062834854479485
drive speeds 0.030346733225820686, 0.2672118848982901, 0.008062834854479485
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.09), Rotation2d(Rads: 0.13, Deg: 7.46))
drive speeds 0.036831976549988585, 0.2649438595705279, 0.007552085954351769
drive speeds 0.036831976549988585, 0.2649438595705279, 0.007552085954351769
drive speeds 0.036831976549988585, 0.2649438595705279, 0.007552085954351769
drive speeds 0.036831976549988585, 0.2649438595705279, 0.007552085954351769
drive speeds 0.036831976549988585, 0.2649438595705279, 0.007552085954351769
drive speeds 0.036831976549988585, 0.2649438595705279, 0.007552085954351769
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.09), Rotation2d(Rads: 0.13, Deg: 7.54))
drive speeds 0.03658788226172509, 0.2642412722250275, 0.007629533488047732
drive speeds 0.03658788226172509, 0.2642412722250275, 0.007629533488047732
drive speeds 0.03658788226172509, 0.2642412722250275, 0.007629533488047732
drive speeds 0.03658788226172509, 0.2642412722250275, 0.007629533488047732
drive speeds 0.03658788226172509, 0.2642412722250275, 0.007629533488047732
drive speeds 0.03658788226172509, 0.2642412722250275, 0.007629533488047732
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.09), Rotation2d(Rads: 0.13, Deg: 7.29))
drive speeds 0.031750479186444254, 0.2610413047593805, 0.0073809203632235415
drive speeds 0.031750479186444254, 0.2610413047593805, 0.0073809203632235415
drive speeds 0.031750479186444254, 0.2610413047593805, 0.0073809203632235415
drive speeds 0.031750479186444254, 0.2610413047593805, 0.0073809203632235415
drive speeds 0.031750479186444254, 0.2610413047593805, 0.0073809203632235415
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.09), Rotation2d(Rads: 0.13, Deg: 7.39))
drive speeds 0.032130236311015914, 0.26345741302238584, 0.007480465002456917
drive speeds 0.032130236311015914, 0.26345741302238584, 0.007480465002456917
drive speeds 0.032130236311015914, 0.26345741302238584, 0.007480465002456917
drive speeds 0.032130236311015914, 0.26345741302238584, 0.007480465002456917
drive speeds 0.032130236311015914, 0.26345741302238584, 0.007480465002456917
drive speeds 0.029234220158355816, 0.2638834558348329, 0.0076007477570481045
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.09), Rotation2d(Rads: 0.13, Deg: 7.51))
drive speeds 0.029234220158355816, 0.2638834558348329, 0.0076007477570481045
drive speeds 0.029234220158355816, 0.2638834558348329, 0.0076007477570481045
drive speeds 0.029234220158355816, 0.2638834558348329, 0.0076007477570481045
drive speeds 0.029234220158355816, 0.2638834558348329, 0.0076007477570481045
drive speeds 0.029234220158355816, 0.2638834558348329, 0.0076007477570481045
drive speeds 0.029234220158355816, 0.2638834558348329, 0.0076007477570481045
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.09), Rotation2d(Rads: 0.13, Deg: 7.26))
drive speeds 0.03000499392074326, 0.26337364465662144, 0.007346629197601908
drive speeds 0.03000499392074326, 0.26337364465662144, 0.007346629197601908
drive speeds 0.03000499392074326, 0.26337364465662144, 0.007346629197601908
drive speeds 0.03000499392074326, 0.26337364465662144, 0.007346629197601908
drive speeds 0.03000499392074326, 0.26337364465662144, 0.007346629197601908
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.09), Rotation2d(Rads: 0.13, Deg: 7.40))
drive speeds 0.03603998944469306, 0.2577807067113087, 0.007495358323596237
drive speeds 0.03603998944469306, 0.2577807067113087, 0.007495358323596237
drive speeds 0.03603998944469306, 0.2577807067113087, 0.007495358323596237
drive speeds 0.03603998944469306, 0.2577807067113087, 0.007495358323596237
drive speeds 0.03603998944469306, 0.2577807067113087, 0.007495358323596237
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.09), Rotation2d(Rads: 0.13, Deg: 7.50))
drive speeds 0.03511407324101018, 0.24802409591168365, 0.0075947484644481415
drive speeds 0.03511407324101018, 0.24802409591168365, 0.0075947484644481415
drive speeds 0.03511407324101018, 0.24802409591168365, 0.0075947484644481415
drive speeds 0.03511407324101018, 0.24802409591168365, 0.0075947484644481415
drive speeds 0.03511407324101018, 0.24802409591168365, 0.0075947484644481415
drive speeds 0.03511407324101018, 0.24802409591168365, 0.0075947484644481415
drive speeds 0.03511407324101018, 0.24802409591168365, 0.0075947484644481415
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.10), Rotation2d(Rads: 0.13, Deg: 7.28))
drive speeds 0.03764844860436519, 0.24276653884577198, 0.007370448526242768
drive speeds 0.03764844860436519, 0.24276653884577198, 0.007370448526242768
drive speeds 0.03764844860436519, 0.24276653884577198, 0.007370448526242768
drive speeds 0.03764844860436519, 0.24276653884577198, 0.007370448526242768
drive speeds 0.03764844860436519, 0.24276653884577198, 0.007370448526242768
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.09), Rotation2d(Rads: 0.13, Deg: 7.55))
drive speeds 0.04044885809916282, 0.24903863836358803, 0.0076471223432872755
drive speeds 0.04044885809916282, 0.24903863836358803, 0.0076471223432872755
drive speeds 0.04044885809916282, 0.24903863836358803, 0.0076471223432872755
drive speeds 0.04044885809916282, 0.24903863836358803, 0.0076471223432872755
drive speeds 0.04044885809916282, 0.24903863836358803, 0.0076471223432872755
drive speeds 0.04044885809916282, 0.24903863836358803, 0.0076471223432872755
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.09), Rotation2d(Rads: 0.13, Deg: 7.71))
drive speeds 0.03455889728802562, 0.25554899218565386, 0.007805762695056291
drive speeds 0.03455889728802562, 0.25554899218565386, 0.007805762695056291
drive speeds 0.03455889728802562, 0.25554899218565386, 0.007805762695056291
drive speeds 0.03455889728802562, 0.25554899218565386, 0.007805762695056291
drive speeds 0.03455889728802562, 0.25554899218565386, 0.007805762695056291
drive speeds 0.03455889728802562, 0.25554899218565386, 0.007805762695056291
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.09), Rotation2d(Rads: 0.14, Deg: 7.90))
drive speeds 0.031296366355771106, 0.2650154362726795, 0.007993754135100787
drive speeds 0.031296366355771106, 0.2650154362726795, 0.007993754135100787
drive speeds 0.031296366355771106, 0.2650154362726795, 0.007993754135100787
drive speeds 0.031296366355771106, 0.2650154362726795, 0.007993754135100787
drive speeds 0.031296366355771106, 0.2650154362726795, 0.007993754135100787
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.09), Rotation2d(Rads: 0.13, Deg: 7.62))
drive speeds 0.024798403910504727, 0.27996190172014723, 0.007718055565093831
drive speeds 0.024798403910504727, 0.27996190172014723, 0.007718055565093831
drive speeds 0.024798403910504727, 0.27996190172014723, 0.007718055565093831
drive speeds 0.024798403910504727, 0.27996190172014723, 0.007718055565093831
drive speeds 0.024798403910504727, 0.27996190172014723, 0.007718055565093831
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.08), Rotation2d(Rads: 0.14, Deg: 7.92))
drive speeds 0.02804095606953265, 0.2808988339278929, 0.008021359847398076
drive speeds 0.02804095606953265, 0.2808988339278929, 0.008021359847398076
drive speeds 0.02804095606953265, 0.2808988339278929, 0.008021359847398076
drive speeds 0.02804095606953265, 0.2808988339278929, 0.008021359847398076
drive speeds 0.02804095606953265, 0.2808988339278929, 0.008021359847398076
drive speeds 0.02804095606953265, 0.2808988339278929, 0.008021359847398076
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.08), Rotation2d(Rads: 0.14, Deg: 7.80))
drive speeds 0.031648992499871416, 0.28136229124396983, 0.007890919582586272
drive speeds 0.031648992499871416, 0.28136229124396983, 0.007890919582586272
drive speeds 0.031648992499871416, 0.28136229124396983, 0.007890919582586272
drive speeds 0.031648992499871416, 0.28136229124396983, 0.007890919582586272
drive speeds 0.031648992499871416, 0.28136229124396983, 0.007890919582586272
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.08), Rotation2d(Rads: 0.12, Deg: 7.15))
drive speeds 0.026831231093978735, 0.2874902007694671, 0.007240236309884537
drive speeds 0.026831231093978735, 0.2874902007694671, 0.007240236309884537
drive speeds 0.026831231093978735, 0.2874902007694671, 0.007240236309884537
drive speeds 0.026831231093978735, 0.2874902007694671, 0.007240236309884537
drive speeds 0.026831231093978735, 0.2874902007694671, 0.007240236309884537
drive speeds 0.026831231093978735, 0.2874902007694671, 0.007240236309884537
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.08), Rotation2d(Rads: 0.13, Deg: 7.48))
drive speeds 0.023819346806396267, 0.2957278244674638, 0.007575348006964902
drive speeds 0.023819346806396267, 0.2957278244674638, 0.007575348006964902
drive speeds 0.023819346806396267, 0.2957278244674638, 0.007575348006964902
drive speeds 0.023819346806396267, 0.2957278244674638, 0.007575348006964902
drive speeds 0.023819346806396267, 0.2957278244674638, 0.007575348006964902
drive speeds 0.023819346806396267, 0.2957278244674638, 0.007575348006964902
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.08), Rotation2d(Rads: 0.14, Deg: 7.86))
drive speeds 0.027776243820886923, 0.2977134441354877, 0.007958539624357056
drive speeds 0.027776243820886923, 0.2977134441354877, 0.007958539624357056
drive speeds 0.027776243820886923, 0.2977134441354877, 0.007958539624357056
drive speeds 0.027776243820886923, 0.2977134441354877, 0.007958539624357056
drive speeds 0.027776243820886923, 0.2977134441354877, 0.007958539624357056
drive speeds 0.027776243820886923, 0.2977134441354877, 0.007958539624357056
drive speeds 0.027776243820886923, 0.2977134441354877, 0.007958539624357056
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.08), Rotation2d(Rads: 0.14, Deg: 8.28))
drive speeds 0.02451093499153829, 0.2817956021246202, 0.008384603474461872
drive speeds 0.02451093499153829, 0.2817956021246202, 0.008384603474461872
drive speeds 0.02451093499153829, 0.2817956021246202, 0.008384603474461872
drive speeds 0.02451093499153829, 0.2817956021246202, 0.008384603474461872
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.08), Rotation2d(Rads: 0.14, Deg: 7.82))
drive speeds 0.03154148918064532, 0.28073400099509127, 0.007913129984592586
drive speeds 0.03154148918064532, 0.28073400099509127, 0.007913129984592586
drive speeds 0.03154148918064532, 0.28073400099509127, 0.007913129984592586
drive speeds 0.03154148918064532, 0.28073400099509127, 0.007913129984592586
drive speeds 0.03154148918064532, 0.28073400099509127, 0.007913129984592586
drive speeds 0.03154148918064532, 0.28073400099509127, 0.007913129984592586
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.09), Rotation2d(Rads: 0.13, Deg: 7.50))
drive speeds 0.02970245806545427, 0.2643287147859722, 0.007597071973387404
drive speeds 0.02970245806545427, 0.2643287147859722, 0.007597071973387404
drive speeds 0.02970245806545427, 0.2643287147859722, 0.007597071973387404
drive speeds 0.02970245806545427, 0.2643287147859722, 0.007597071973387404
drive speeds 0.02970245806545427, 0.2643287147859722, 0.007597071973387404
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.10), Rotation2d(Rads: 0.11, Deg: 6.37))
drive speeds 0.02074474370669427, 0.23646555838424152, 0.006448638656406729
drive speeds 0.02074474370669427, 0.23646555838424152, 0.006448638656406729
drive speeds 0.02074474370669427, 0.23646555838424152, 0.006448638656406729
drive speeds 0.02074474370669427, 0.23646555838424152, 0.006448638656406729
drive speeds 0.02074474370669427, 0.23646555838424152, 0.006448638656406729
drive speeds 0.02074474370669427, 0.23646555838424152, 0.006448638656406729
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.11), Rotation2d(Rads: 0.11, Deg: 6.50))
drive speeds 0.021729637019377645, 0.20843141312758515, 0.006584269319825229
drive speeds 0.021729637019377645, 0.20843141312758515, 0.006584269319825229
drive speeds 0.021729637019377645, 0.20843141312758515, 0.006584269319825229
drive speeds 0.021729637019377645, 0.20843141312758515, 0.006584269319825229
drive speeds 0.021729637019377645, 0.20843141312758515, 0.006584269319825229
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.12), Rotation2d(Rads: 0.10, Deg: 5.79))
drive speeds 0.023716259133356798, 0.18052670943816862, 0.0058630163102828285
drive speeds 0.023716259133356798, 0.18052670943816862, 0.0058630163102828285
drive speeds 0.023716259133356798, 0.18052670943816862, 0.0058630163102828285
drive speeds 0.023716259133356798, 0.18052670943816862, 0.0058630163102828285
drive speeds 0.023716259133356798, 0.18052670943816862, 0.0058630163102828285
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.13), Rotation2d(Rads: 0.09, Deg: 4.88))
drive speeds 0.01685201932232643, 0.1375818215330622, 0.004935431528537111
drive speeds 0.01685201932232643, 0.1375818215330622, 0.004935431528537111
drive speeds 0.01685201932232643, 0.1375818215330622, 0.004935431528537111
drive speeds 0.01685201932232643, 0.1375818215330622, 0.004935431528537111
drive speeds 0.01685201932232643, 0.1375818215330622, 0.004935431528537111
drive speeds 0.01685201932232643, 0.1375818215330622, 0.004935431528537111
robot pose to target NWU Pose2d(Translation2d(X: -0.65, Y: -0.14), Rotation2d(Rads: 0.07, Deg: 4.17))
drive speeds 0.016411314735045358, 0.08933364905924945, 0.0042214278608619085
drive speeds 0.016411314735045358, 0.08933364905924945, 0.0042214278608619085
drive speeds 0.016411314735045358, 0.08933364905924945, 0.0042214278608619085
drive speeds 0.016411314735045358, 0.08933364905924945, 0.0042214278608619085
drive speeds 0.016411314735045358, 0.08933364905924945, 0.0042214278608619085
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.07, Deg: 3.98))
drive speeds 0.020112371197070343, 0.07864353132323007, 0.004028606817100484
drive speeds 0.020112371197070343, 0.07864353132323007, 0.004028606817100484
drive speeds 0.020112371197070343, 0.07864353132323007, 0.004028606817100484
drive speeds 0.020112371197070343, 0.07864353132323007, 0.004028606817100484
drive speeds 0.020112371197070343, 0.07864353132323007, 0.004028606817100484
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.06, Deg: 3.65))
drive speeds 0.018514556365441803, 0.0634309263093281, 0.0036983472300216665
drive speeds 0.0, 0.0, 0.0
drive speeds 0.018514556365441803, 0.0634309263093281, 0.0036983472300216665
drive speeds 0.0, 0.0, 0.0
drive speeds 0.018514556365441803, 0.0634309263093281, 0.0036983472300216665
drive speeds 0.0, 0.0, 0.0
drive speeds 0.018514556365441803, 0.0634309263093281, 0.0036983472300216665
drive speeds 0.0, 0.0, 0.0
drive speeds 0.018514556365441803, 0.0634309263093281, 0.0036983472300216665
drive speeds 0.0, 0.0, 0.0
drive speeds 0.018514556365441803, 0.0634309263093281, 0.0036983472300216665
drive speeds 0.0, 0.0, 0.0
drive speeds 0.018514556365441803, 0.0634309263093281, 0.0036983472300216665
drive speeds 0.0, 0.0, 0.0
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.07, Deg: 4.03))
drive speeds 0.020083439201548602, 0.04219271595456725, 0.004076128883481962
drive speeds 0.0, 0.0, 0.0
drive speeds 0.020083439201548602, 0.04219271595456725, 0.004076128883481962
drive speeds 0.0, 0.0, 0.0
drive speeds 0.020083439201548602, 0.04219271595456725, 0.004076128883481962
drive speeds 0.0, 0.0, 0.0
drive speeds 0.020083439201548602, 0.04219271595456725, 0.004076128883481962
drive speeds 0.0, 0.0, 0.0
drive speeds 0.020083439201548602, 0.04219271595456725, 0.004076128883481962
drive speeds 0.0, 0.0, 0.0
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.06, Deg: 3.44))
drive speeds 0.019369716713960206, 0.026015585547768262, 0.0034828050169140815
drive speeds 0.0, 0.0, 0.0
drive speeds 0.019369716713960206, 0.026015585547768262, 0.0034828050169140815
drive speeds 0.0, 0.0, 0.0
drive speeds 0.019369716713960206, 0.026015585547768262, 0.0034828050169140815
drive speeds 0.0, 0.0, 0.0
drive speeds 0.019369716713960206, 0.026015585547768262, 0.0034828050169140815
drive speeds 0.0, 0.0, 0.0
drive speeds 0.0, 0.0, 0.0
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.17), Rotation2d(Rads: 0.04, Deg: 2.34))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.17), Rotation2d(Rads: 0.04, Deg: 2.10))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.17), Rotation2d(Rads: 0.03, Deg: 1.73))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.18), Rotation2d(Rads: 0.02, Deg: 1.36))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.18), Rotation2d(Rads: 0.04, Deg: 2.18))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.18), Rotation2d(Rads: 0.01, Deg: 0.83))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.18), Rotation2d(Rads: 0.02, Deg: 1.36))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.17), Rotation2d(Rads: 0.03, Deg: 1.79))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.17), Rotation2d(Rads: 0.03, Deg: 1.91))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.17), Rotation2d(Rads: 0.03, Deg: 1.77))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.04, Deg: 2.11))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.04, Deg: 2.09))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.03, Deg: 1.49))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.04, Deg: 2.37))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.04, Deg: 2.49))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.04, Deg: 2.47))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.05, Deg: 2.78))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.05, Deg: 2.90))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.05, Deg: 2.74))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.16), Rotation2d(Rads: 0.06, Deg: 3.64))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.06, Deg: 3.36))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.15), Rotation2d(Rads: 0.07, Deg: 4.06))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.14), Rotation2d(Rads: 0.07, Deg: 4.01))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.13), Rotation2d(Rads: 0.07, Deg: 3.93))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.13), Rotation2d(Rads: 0.06, Deg: 3.48))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.13), Rotation2d(Rads: 0.08, Deg: 4.44))
robot pose to target NWU Pose2d(Translation2d(X: -0.66, Y: -0.12), Rotation2d(Rads: 0.08, Deg: 4.54))
robot pose to target NWU Pose2d(Translation2d(X: -0.67, Y: -0.12), Rotation2d(Rads: 0.08, Deg: 4.68))
robot pose to target NWU Pose2d(Translation2d(X: -0.68, Y: -0.13), Rotation2d(Rads: 0.07, Deg: 4.01))
robot pose to target NWU Pose2d(Translation2d(X: -0.69, Y: -0.12), Rotation2d(Rads: 0.06, Deg: 3.66))
robot pose to target NWU Pose2d(Translation2d(X: -0.71, Y: -0.13), Rotation2d(Rads: 0.02, Deg: 1.42))
robot pose to target NWU Pose2d(Translation2d(X: -0.71, Y: -0.12), Rotation2d(Rads: 0.03, Deg: 1.46))
robot pose to target NWU Pose2d(Translation2d(X: -0.71, Y: -0.13), Rotation2d(Rads: 0.00, Deg: 0.27))
robot pose to target NWU Pose2d(Translation2d(X: -0.00, Y: -0.00), Rotation2d(Rads: -0.00, Deg: -0.00))
robot pose to target NWU Pose2d(Translation2d(X: -0.79, Y: -0.08), Rotation2d(Rads: -0.03, Deg: -1.70))
robot pose to target NWU Pose2d(Translation2d(X: -0.79, Y: -0.09), Rotation2d(Rads: -0.02, Deg: -1.36))
robot pose to target NWU Pose2d(Translation2d(X: -0.79, Y: -0.09), Rotation2d(Rads: -0.03, Deg: -1.67))
robot pose to target NWU Pose2d(Translation2d(X: -0.79, Y: -0.08), Rotation2d(Rads: -0.02, Deg: -1.09))
robot pose to target NWU Pose2d(Translation2d(X: -0.79, Y: -0.09), Rotation2d(Rads: -0.00, Deg: -0.11))
robot pose to target NWU Pose2d(Translation2d(X: -0.79, Y: -0.09), Rotation2d(Rads: -0.00, Deg: -0.16))
robot pose to target NWU Pose2d(Translation2d(X: -0.79, Y: -0.09), Rotation2d(Rads: -0.03, Deg: -1.74))
robot pose to target NWU Pose2d(Translation2d(X: -0.79, Y: -0.09), Rotation2d(Rads: -0.03, Deg: -1.89))
robot pose to target NWU Pose2d(Translation2d(X: -0.79, Y: -0.09), Rotation2d(Rads: -0.04, Deg: -2.05))
robot pose to target NWU Pose2d(Translation2d(X: -0.79, Y: -0.09), Rotation2d(Rads: -0.03, Deg: -1.86))
robot pose to target NWU Pose2d(Translation2d(X: -0.79, Y: -0.09), Rotation2d(Rads: -0.04, Deg: -2.16))
robot pose to target NWU Pose2d(Translation2d(X: -0.79, Y: -0.09), Rotation2d(Rads: -0.03, Deg: -1.88))
robot pose to target NWU Pose2d(Translation2d(X: -0.79, Y: -0.09), Rotation2d(Rads: -0.04, Deg: -2.04))
robot pose to target NWU Pose2d(Translation2d(X: -0.79, Y: -0.09), Rotation2d(Rads: -0.03, Deg: -1.89))
robot pose to target NWU Pose2d(Translation2d(X: -0.79, Y: -0.09), Rotation2d(Rads: -0.04, Deg: -2.01))
robot pose to target NWU Pose2d(Translation2d(X: -0.79, Y: -0.09), Rotation2d(Rads: -0.03, Deg: -1.83))
robot pose to target NWU Pose2d(Translation2d(X: -0.79, Y: -0.09), Rotation2d(Rads: -0.04, Deg: -2.21))
robot pose to target NWU Pose2d(Translation2d(X: -0.79, Y: -0.09), Rotation2d(Rads: -0.03, Deg: -1.98))
robot pose to target NWU Pose2d(Translation2d(X: -0.79, Y: -0.09), Rotation2d(Rads: -0.04, Deg: -2.07))
robot pose to target NWU Pose2d(Translation2d(X: -0.79, Y: -0.09), Rotation2d(Rads: -0.04, Deg: -2.04))
robot pose to target NWU Pose2d(Translation2d(X: -0.79, Y: -0.09), Rotation2d(Rads: -0.03, Deg: -1.90))
robot pose to target NWU Pose2d(Translation2d(X: -0.79, Y: -0.09), Rotation2d(Rads: -0.03, Deg: -1.96))
robot pose to target NWU Pose2d(Translation2d(X: -0.79, Y: -0.09), Rotation2d(Rads: -0.03, Deg: -1.92))
robot pose to target NWU Pose2d(Translation2d(X: -0.79, Y: -0.09), Rotation2d(Rads: -0.03, Deg: -1.83))
 */