package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

public class PhotonVision {

      // Tag positions
    // tag rotation is CCW looking down on field from the ceiling.
    // rotating around Z, 0 degrees is parallel to Y and facing down field or +X. 30 degrees is still
    // facing down field +X and a little facing into the +Y across the field

    final boolean CustomTagLayout = false; // true is use custom deploy of layout

    AprilTagFieldLayout aprilTagFieldLayout;

    // Change this to match the name of your camera
    PhotonCamera camera = new PhotonCamera("Arducam_OV9281_USB_Camera_(A)");
    // Microsoft_LifeCam_HD-3000   Arducam_OV9281_USB_Camera_(B)

    //Forward Camera
    //Cam mounted facing forward, half a meter forward of center, half a meter up from center.
    Transform3d robotToCamera = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0));

    PhotonPoseEstimator photonPoseEstimator;

    public PhotonVision() {
      try { // duplicated from Robot.java
        if(CustomTagLayout)
          aprilTagFieldLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/2024-crescendo.json"); // custom file example
        else
        // aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(); // 2024 syntax
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
      } catch (IOException e) {
        e.printStackTrace();
        aprilTagFieldLayout = null;
      }
  
      PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCamera);

    }

    public void PVacquire() {

        var resultAll = camera.getAllUnreadResults();

        if (resultAll.isEmpty()) {
          System.out.println("\nno PV result");
        }
        else {
          int countResults = 0;

          for (PhotonPipelineResult result : resultAll) {
            System.out.println("result #" + ++countResults);
            // Check if the latest result has any targets.
            boolean hasTargets = result.hasTargets();

            // Get a list of currently tracked targets.
            List<PhotonTrackedTarget> targets = result.getTargets();

            // Get the current best target.
            PhotonTrackedTarget target = result.getBestTarget();

            // Get information from target.
            double yaw = target.getYaw();
            double pitch = target.getPitch();
            double area = target.getArea();
            double skew = target.getSkew();
            Transform3d pose = target.getBestCameraToTarget();
            List<TargetCorner> corners = target.getDetectedCorners();

            // Get information from target.
            int targetID = target.getFiducialId();
            double poseAmbiguity = target.getPoseAmbiguity();
            Transform3d bestCameraToTarget = target.getBestCameraToTarget();
            Transform3d alternateCameraToTarget = target.getAlternateCameraToTarget();

    // https://docs.photonvision.org/en/v2025.0.0-beta-4/docs/apriltag-pipelines/multitag.html
    // Enabling MultiTag

            if (result.getMultiTagResult().isPresent()) {
              Transform3d fieldToCamera = result.getMultiTagResult().get().estimatedPose.best;
            }

            System.out.println("\nRESULT " + result + "\n\nBEST TARGET " + target);
          }
      }
    }

//FIXME
// call this then use it in addVisionMeasurement
// https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html
    // public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    //     photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
    //     return photonPoseEstimator.update(); //FIXME need "update(result)" - which one if multiple; also need previous pose passed in
    // }
}

/*
no PV results
result #1

RESULT PhotonPipelineResult [metadata=PhotonPipelineMetadata [captureTimestampMicros=9628776, publishTimestampMicros=9652431, sequenceID=31692, timeSinceLastPong=974440], targets=[PhotonTrackedTarget [yaw=15.025610619756835, pitch=-1.967468576919871, area=1.5, skew=0.0, fiducialId=7, objDetectId=-1, objDetectConf=-1.0, bestCameraToTarget=Transform3d(Translation3d(X: 0.00, Y: 0.00, Z: 0.00), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0))), altCameraToTarget=Transform3d(Translation3d(X: 0.00, Y: 0.00, Z: 0.00), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0))), poseAmbiguity=-1.0, minAreaRectCorners=[(818.565491164281,417.0119688597161), (858.4179621716057,352.7121863282917), (1056.852477585719,475.70025037856516), (1017.0000065783943,540.0000329099896)], detectedCorners=[(825.3000488281252,421.50106811523443), (1017.5325927734374,540.2463989257812), (1017.7407226562499,451.86822509765625), (857.4851684570314,355.47784423828125)]]], multitagResult=Optional.empty]

BEST TARGET PhotonTrackedTarget [yaw=15.025610619756835, pitch=-1.967468576919871, area=1.5, skew=0.0, fiducialId=7, objDetectId=-1, objDetectConf=-1.0, bestCameraToTarget=Transform3d(Translation3d(X: 0.00, Y: 0.00, Z: 0.00), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0))), altCameraToTarget=Transform3d(Translation3d(X: 0.00, Y: 0.00, Z: 0.00), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0))), poseAmbiguity=-1.0, minAreaRectCorners=[(818.565491164281,417.0119688597161), (858.4179621716057,352.7121863282917), (1056.852477585719,475.70025037856516), (1017.0000065783943,540.0000329099896)], detectedCorners=[(825.3000488281252,421.50106811523443), (1017.5325927734374,540.2463989257812), (1017.7407226562499,451.86822509765625), (857.4851684570314,355.47784423828125)]]
*/

// C:\Users\bike1\downloads>java -jar photonvision-v2025.0.0-beta-6-winx64.jar