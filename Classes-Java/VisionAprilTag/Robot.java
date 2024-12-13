// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


/**
 * This is a demo program showing the detection of AprilTags. The image is acquired from the USB
 * camera, then any detected AprilTags are marked up on the image, transformed to the robot on
 * the field pose and sent to the dashboard.
 *
 * Be aware that the performance on this is much worse than a coprocessor solution!
 * 
 * This example includes an estimated latency time from the time the image appears through to
 * the end of processing the robot pose.
 * 
 * The latency of acquiring the image from the camera mostly depends on the fps setting
 * of the camera, the shutter is global or where the object is if progressive scan.
 * Timing starts before the frame grab statement and that appears to account for much
 * of the camera latency at least for the LifeCam.
 * 
 * The camera view is displayed with additional information of latency and the AprilTag pose to camera.
 * That display is optional and a tiny bit of cpu processing can be saved by not doing it.
 * Since AprilTag view can be in normal light that camera can also be used by the operator if it's
 * pointing in a good direction. If the operator doesn't need that view, don't display it and
 * the image can be made a little darker and more contrast - whatever can reduce the cpu
 * processing time for an image. Experiment with exposure, contrast, gamma, brightness, etc.
 * 
 * Apriltag has known pose on the field loaded from file (WPILib or your custom file).
 * Detected tag's perspective seen by the camera is used to calculate an estimate of the camera pose relative to the tag.
 * Camera has a known pose relative to the robot chassis hard coded herein (change it!).
 * Combine this chain to calculate the robot pose in the field.
 * Camera parameters must be provided from another source source as the related calibration program.
*/     

package frc.robot;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDouble;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.MatOfPoint3f;
import org.opencv.core.Point;
import org.opencv.core.Point3;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagDetection;
import edu.wpi.first.apriltag.AprilTagDetector;
import edu.wpi.first.apriltag.AprilTagDetector.QuadThresholdParameters;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.apriltag.AprilTagPoseEstimator;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.CvSource;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.ComputerVisionUtil;
import edu.wpi.first.math.geometry.CoordinateSystem;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  
  enum CameraOption{ArduCam320x240, ArduCam1280x800, LifeCam320x240, LifeCam640x480};

  //FIXME select your camera from the list above
  private final CameraOption selectCameraOption = CameraOption.ArduCam1280x800;

  static {
    System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
}

  // the roboRIO won't handle more resolution than about 320x240 (not enough cpu).
  // Calibrate the camera at the used resolution or scale Fx,Fy,Cx,Cy proportional
  // to what resolution was used for camera calibration.
  // fx camera horizontal focal length, in pixels
  // fy camera vertical focal length, in pixels
  // cx camera horizontal focal center, in pixels
  // cy camera vertical focal center, in pixels

  int cameraW;
  int cameraH;
  int fps;
  double cameraFx;
  double cameraFy;
  double cameraCx;
  double cameraCy;
  MatOfDouble distortionCoeffs;

  public Image image = new Image(); // where a video frame goes for multiple processes to use

  //FIXME select LL usage
  final boolean useLL = true; // do LimeLight processing
  LL ll;

  public Robot() {

    switch(selectCameraOption) {
      case ArduCam320x240: // rough calibration - wasn't done with a nice flat board
        cameraW = 320;
        cameraH = 240;
        fps = 100;
        cameraFx = 273.8682279422785;
        cameraFy = 274.2578211409246;
        cameraCx = 142.187975375679;
        cameraCy = 124.6151823259089;
        distortionCoeffs = new MatOfDouble();
        // much of the small amount of distortion from calibration was actually the board
        // not being smooth so don't bother using the distortion
        // [0.03872533667096114, -0.2121025605447465, 0.00334472765894009, -0.006080540135581289, 0.4001779842036727]
        break;

      case ArduCam1280x800:// Arducam_OV9281_USB_Camera_(A?); Cameron board 1280x800      
        cameraW = 1280;
        cameraH = 800;
        fps = 100;
        cameraFx = 907.6920444758049;
        cameraFy = 907.1513951038395;
        cameraCx = 604.1750223777503;
        cameraCy = 416.4609913313957;
        distortionCoeffs = new MatOfDouble(
          0.040354289830866516, -0.044066115475547216, 6.662818829158613E-4,9.755603732755772E-4,  // k1 k2 p1 p2
            -0.013630390510289322, // k3
            -0.0011985508423857224, 0.003370423168524356, 0.0010337869630847195); // k4 k5 k6
            // assume s1 s2 s3 s4 tx ty are all zeros
        break;

      case LifeCam320x240:
        // 320x240 lifecam calibration from PhotonVision
        cameraW = 320;
        cameraH = 240;
        fps = 30;
        cameraFx = 353.74653217742724;
        cameraFy = 340.77624878700817;
        cameraCx = 163.5540798921191;
        cameraCy = 119.8945718300403;
        distortionCoeffs = new MatOfDouble();
        break;

      case LifeCam640x480:
        // 640x480 lifecam calibration from WPILib example
        cameraW = 640;
        cameraH = 480;
        fps = 30;
        cameraFx = 699.3778103158814;
        cameraFy = 677.7161226393544;
        cameraCx = 345.6059345433618;
        cameraCy = 207.12741326228522;
        distortionCoeffs = new MatOfDouble();
        break;

      default: break;

    }

    // Set up Pose Estimator - parameters are for a Microsoft Lifecam HD-3000
    // (https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21)
    
    // theoretically the resolution factor also directly effects the other camera parameters
    // but apparently recalibrating at various resolutions does yield slightly varying results.

    // the other arducam with Cameron board
    // 902.229862551387, 0.0, 630.0164752216564,
    // 0.0, 901.4463862207839, 413.07035838353374,
    // 0.0, 0.0, 1.0
        
    // using Samsung tablet
    // camMatrix: 
    // [908.014390012331, 0.0, 597.8814589360088,
    // 0.0, 908.6191476725982, 413.8612971221383,
    //  0.0, 0.0, 1.0]
    // distortionCoeffs:
    // [0.052173675342917676, -0.07090132559762727, -5.866285222375848E-4, -0.002475170189898108, 0.012835458369393258, -0.002442550780537089, 0.005625375255529692, 9.391199615674332E-4]

    // using samsung desktop screen
    // CALIBRATION SUCCESS for res 1280x800 in 291.6018ms! camMatrix: 
    // [907.5869786612509, 0.0, 604.8488080504388, 0.0, 909.4739674568915, 411.6013865871937, 0.0, 0.0, 1.0]
    // distortionCoeffs:
    //  0.05483635427166114, k1
    // -0.09776533602636311, k2
    // -9.78236645068978E-5, p1
    //  4.8823267235786116E-5, p2
    //  0.04332976089574157, k3
    // -0.00373311108258594, k4
    //  0.005395356105023349, k5
    // -1.99252152117131E-4 k6
    // (1 + 0.05483635427166114r^2 + -0.09776533602636311r^4 + 0.04332976089574157r^6) / (1 + -0.00373311108258594r^2 + 0.005395356105023349r^4 + -1.99252152117131E-4r^6) for r from 0 to 400

    //opencv tutorial example
    // https://www.wolframalpha.com/input/?i=plot (1 + -4.1802327176423804e-001 Power[\(40)Divide[r,6.5746697944293521e+002]\(41),2] + 5.0715244063187526e-001 Power[\(40)Divide[r,6.5746697944293521e+002]\(41),4] + -5.7843597214487474e-001*Power[\(40)Divide[r,6.5746697944293521e+002]\(41),6]) 
    // all times r
    // for r from 0 to 400

    var visionThread1 = new Thread(this::acquireApriltagThread);
    visionThread1.setDaemon(true);
    visionThread1.start();

    // make sure input camera starts before output stream to get the 1181 and 1182 in the right view order
    try {
      Thread.sleep(4000);
    }
    catch (InterruptedException e) {
      e.printStackTrace();
    }

    var visionThread2 = new Thread(this::acquireRobotPoseThread);
    visionThread2.setDaemon(true);
    visionThread2.start();

    if (useLL) ll = new LL();
  }

  @Override
  public void robotPeriodic()
  {
      if (useLL) ll.LLacquire();
  }

  @Override
  public void teleopPeriodic()
  // test how much cpu time is left on roboRIO 1 after AprilTag pose process
  // enable this increases latency from 80ish to 95ish [ms]
  // and reduces fps from 20ish to 17ish (both have very wide variations)
  {
    // for(int i = 1; i < 600_000; i++); // waste some cputime - the edge of 20ms overruns
  }

  void acquireApriltagThread() {
    int frameNumber = 0;
    long acquisitionTime = 0;
    long frameError = 0;
    
    var detector = new AprilTagDetector();
    // // look for tag16h5 2023, don't correct any error bits
    // detector.addFamily("tag16h5", 0); 2023

    // look for tag36h11 2024, correct 0 or 1 error bits on roboRIO v1 or up to 2 on roboRIO v2
    detector.addFamily("tag36h11", 1);

    // 2025 beta 1 & 2 had 300 default which is too large to be useful so use 2024 value of 5
    QuadThresholdParameters qtp = detector.getQuadThresholdParameters();
    qtp.minClusterPixels = 5;
    // qtp.criticalAngle = 10 * Math.PI / 180.0; // also changed in 2025 beta 1 for unknown reasons
    detector.setQuadThresholdParameters(qtp);
    
    // Get the UsbCamera from CameraServer
    //FIXME pick your camera
    int cameraDeviceId = 0;
    UsbCamera camera = CameraServer.startAutomaticCapture(cameraDeviceId); // http://10.42.37.2:1181/   http://roborio-4237-frc.local:1181/?action=stream
          //"myCam", "/dev/v4l/by-id/usb-Arducam_Technology_Co.__Ltd._Arducam_OV9281_USB_Camera_UC762-video-index0"
    // Set the resolution and frames per second
    camera.setResolution(cameraW, cameraH);
    camera.setFPS(fps);

    // camera.setExposureAuto();

    // Get a CvSink. This will capture Mats from the camera
    CvSink cvSink = CameraServer.getVideo(camera);

    // Mats are very memory expensive. Lets reuse these.
    var mat = new Mat();
    var grayMat = new Mat();

    // This 'while' cannot be 'true'. The program will never exit if it is. Using
    // interrupted() lets the robot stop this thread when restarting robot code or deploying.
    while (!Thread.interrupted()) {
      // Tell the CvSink to grab a frame from the camera and put it
      // in the source mat.  If there is an error notify the output.
      frameNumber++;

      acquisitionTime = System.nanoTime();
      if (cvSink.grabFrame(mat, 1.) == frameError) {
        // Send the output the error.
        System.out.println(cvSink.getError());
        // skip the rest of the current iteration
        continue;
      }

      Imgproc.cvtColor(mat, grayMat, Imgproc.COLOR_BGR2GRAY); // color camera 3 channels to 1 gray channel
   
      // Core.extractChannel(mat, grayMat, 0); // monochrome camera on 3 channels to 1 gray channel (isn't significantly better than above) 

      AprilTagDetection[] detections = detector.detect(grayMat);

      // new Image with detections available for use so pass it on
      image.setImage(mat, detections, acquisitionTime, frameNumber);
      } // end while grab camera frame
      detector.close();
    }

  public void acquireRobotPoseThread()
  {
    // Set up Pose Estimator - parameters included for a Microsoft Lifecam HD-3000
    // and rough estimates for an ArduCam UC-844
    // (https://www.chiefdelphi.com/t/wpilib-apriltagdetector-sample-code/421411/21)

    // Tag positions
    // tag rotation is CCW looking down on field from the ceiling.
    // rotating around Z, 0 degrees is parallel to Y and facing down field or +X. 30 degrees is still
    // facing down field +X and a little facing into the +Y across the field

    final boolean CustomTagLayout = false; // true is use custom deploy of layout

    AprilTagFieldLayout aprilTagFieldLayout;
    try {
      if(CustomTagLayout)
        aprilTagFieldLayout = new AprilTagFieldLayout(Filesystem.getDeployDirectory() + "/2024-crescendo.json"); // custom file example
      else
      // aprilTagFieldLayout = AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(); // 2024 syntax
      aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2024Crescendo);
    } catch (IOException e) {
      e.printStackTrace();
      aprilTagFieldLayout = null;
    }

    // We'll output to NT
    int maxTagId = 25    +1; // add 1 for tag ID 0
    NetworkTable tagsTable = NetworkTableInstance.getDefault().getTable("apriltags");
    IntegerArrayPublisher pubTags = tagsTable.getIntegerArrayTopic("tags").publish();

    List<StructPublisher<Pose3d>> publishRobotPose = new ArrayList<>(maxTagId);
    List<StructPublisher<Pose3d>> publishTagPose = new ArrayList<>(maxTagId);

    // make an empty bucket for every possible tag
    for (int tag = 0; tag < maxTagId; tag++) {
      publishRobotPose.add(null);
      publishTagPose.add(null);
    }

    Consumer<AprilTag> initializeTagAndPose = tag ->
      {
        System.out.format("%s %6.1f, %6.1f, %6.1f [degrees]%n",
              tag.toString(),
              Units.radiansToDegrees(tag.pose.getRotation().getX()),
              Units.radiansToDegrees(tag.pose.getRotation().getY()),
              Units.radiansToDegrees(tag.pose.getRotation().getZ()));

              var robotPosePublisher = tagsTable.getStructTopic("robotPose3D_" + tag.ID, Pose3d.struct).publish();
              var tagPosePublisher = tagsTable.getStructTopic("tagPose3D_" + tag.ID, Pose3d.struct).publish();
              publishRobotPose.set(tag.ID, robotPosePublisher);
              publishTagPose.set(tag.ID, tagPosePublisher);
      };

    System.out.println(aprilTagFieldLayout.getTags().size() + " Tags on file");    
    aprilTagFieldLayout.getTags().forEach(initializeTagAndPose);

    // Instantiate once
    ArrayList<Long> tags = new ArrayList<>(maxTagId);
    var outlineColor = new Scalar(0, 255, 0); // bgr
    var crossColor = new Scalar(0, 0, 255); // bgr
    var crossLength = 10;
    Mat mat = new Mat();
    AcquisitionTime acquisitionTime = new AcquisitionTime();
    int latency = 0;

    // Setup a CvSource. This will send images back to the Dashboard
    CvSource outputStream = CameraServer.putVideo("Detected", cameraW, cameraH); // http://10.42.37.2:1182/  http://roborio-4237-frc.local:1182/?action=stream

    double tagSize = 0.1651; // meters of the targeted AprilTag

    var poseEstConfig =
        new AprilTagPoseEstimator.Config( tagSize, cameraFx, cameraFy, cameraCx, cameraCy);

    var estimator = new AprilTagPoseEstimator(poseEstConfig);

    // while conditional cannot be 'true' otherwise the program will never exit if it is.
    // This lets the robot stop this thread when restarting robot code or deploying.

    // loop to get camera frames
    while (!Thread.interrupted()) {
      // long frameStartTime = System.nanoTime();
    AprilTagDetection[] detections = image.getImage(mat, acquisitionTime); // get the buffered image w/ detections
 
    // have not seen any tags yet
    tags.clear();

    // loop to get all AprilTag detections within current camera frame
    for (AprilTagDetection detection : detections) {

      Pose3d tagInFieldFrame; // pose from WPILib resource or custom pose file

      if(aprilTagFieldLayout.getTagPose(detection.getId()).isPresent() && detection.getDecisionMargin() > 50.) // margin < 20 seems bad; margin > 120 are good
      {
        tagInFieldFrame = aprilTagFieldLayout.getTagPose(detection.getId()).get();
      }
      else
      {
        System.out.println("bad id " + detection.getId() + " " + detection.getDecisionMargin());
        continue;
      }

      // remember we saw this tag
      tags.add((long) detection.getId());

      // draw lines around the tag
      for (var i = 0; i <= 3; i++) {
        var j = (i + 1) % 4;
        var pt1 = new Point(detection.getCornerX(i), detection.getCornerY(i));
        var pt2 = new Point(detection.getCornerX(j), detection.getCornerY(j));
        Imgproc.line(mat, pt1, pt2, outlineColor, 2);
        // corners appear as 3 2
        //                   0 1
      }

      // mark the center of the tag
      var tagCx = detection.getCenterX();
      var tagCy = detection.getCenterY();

      // Calculates the yaw and pitch this tag's center vs the camera principal point.
      // Yaw and pitch must be calculated together to account for perspective distortion.
      // Yaw is positive right and pitch is positive up.
      double yaw = Math.atan((tagCx - cameraCx) / cameraFx);
      double pitch = Math.atan((cameraCy - tagCy) / (cameraFy / Math.cos(yaw)));
      SmartDashboard.putNumber("tag " + detection.getId() + " x angle [deg]", Units.radiansToDegrees(yaw));
      SmartDashboard.putNumber("tag " + detection.getId() + " y angle [deg]", Units.radiansToDegrees(pitch));

      Imgproc.line(mat, new Point(tagCx - crossLength, tagCy), new Point(tagCx + crossLength, tagCy), crossColor, 2);
      Imgproc.line(mat, new Point(tagCx, tagCy - crossLength), new Point(tagCx, tagCy + crossLength), crossColor, 2);

      // identify the tag
      Imgproc.putText(
          mat,
          Integer.toString(detection.getId()),
          new Point(tagCx + crossLength, tagCy),
          Imgproc.FONT_HERSHEY_SIMPLEX,
          1,
          crossColor,
          3);

      // determine pose
      Transform3d tagFacingCameraFrame = estimator.estimate(detection);

      { // draw a frustum in front of the AprilTag
        // use the estimated pose from above before any other transforms  

        // camera same as above but different format for OpenCV
        float[] cameraParm = {(float)cameraFx,   0.f,             (float)cameraCx,
                                0.f,             (float)cameraFy, (float)cameraCy,
                                0.f,             0.f,             1.f};
        Mat K = new Mat(3, 3, CvType.CV_32F); // camera matrix
        K.put(0, 0, cameraParm);

        // 3D points of ideal, original corners, flat on the tag, scaled to the actual tag size.
        // Order doesn't matter except must be in same order as the top points so pillars connect right.
        // We could reuse the corners from detector if we know the order (and we do) and avoid redundant
        // variable and recalculation but we'll re-specify them for fun and match the detectors corners order.
        MatOfPoint3f bottom = new MatOfPoint3f(
          new Point3(-1.*tagSize/2.,1.*tagSize/2., 0.),
              new Point3(1.*tagSize/2., 1.*tagSize/2., 0.),
              new Point3(1.*tagSize/2., -1.*tagSize/2., 0.),
              new Point3(-1.*tagSize/2., -1.*tagSize/2., 0.));

        // 3D points of the ideal, original corners, in front of the tag to make a frustum, scaled to the actual tag size
        // note that the orientation and size of the face of the box can be controlled by the sign of the "Z"
        // value of the "top" variable.
        // "-" (negative) gives larger top facing straight away from the plane of the tag
        // "+" (positive) gives smaller top facing toward the camera
        MatOfPoint3f top = new MatOfPoint3f( // order doesn't matter except must be in same order as the bottom points
          new Point3(-1.*tagSize/2.,1.*tagSize/2., -0.7*tagSize),
              new Point3(1.*tagSize/2., 1.*tagSize/2., -0.7*tagSize),
              new Point3(1.*tagSize/2., -1.*tagSize/2., -0.7*tagSize),
              new Point3(-1.*tagSize/2., -1.*tagSize/2., -0.7*tagSize));

        // The OpenCV rvec is a rotation vector with three elements representing the axis scaled by
        // the angle in the EDN coordinate system. (angle = norm, and axis = rvec / norm).
        // Those three elements are not the same 3 elements of the 3 rotations for the 3 axes that might be used in WPILib Rotation3D.
        // Those are roll, pitch, and yaw (Euler angles?) that can be converted to the rotation vector (directional vector?)
        // and vice versa. The roll, pitch, and yaw angles can be recovered from the rotation vector:
        // Rodrigues conversion of rotation vector to rotation matrix (or vice versa)
        // Rotation matrix to Euler angles https://learnopencv.com/rotation-matrix-to-euler-angles/
        // https://en.wikipedia.org/wiki/Axis%E2%80%93angle_representation
        /* https://www.reddit.com/r/computervision/comments/eek49u/q_how_to_get_the_euler_angles_from_a_rotation/?rdt=45312
        I think of the compact representations of a rotation (Euler angles, Rodrigues vector, quaternion, etc) as practical
        ways to communicate or store the information about rotation. But when it comes to actually using it for something 99 percent
        of the time I use the good ol 3x3 rotation matrix because it's super easy to understand. multiply a vector and you rotate
        the vector, invert it and get the rotation the other way round, want to know the base vectors of the world frame of
        reference wrt the camera? this are the columns of the rotation matrix.
        */
        double[] rotationVector = tagFacingCameraFrame.getRotation().getQuaternion().toRotationVector().getData(); // 3x1 3 rows 1 col

        Mat T = new Mat(3, 1, CvType.CV_64FC1);
        Mat R = new Mat(3, 1, CvType.CV_64FC1);
        T.put(0, 0, tagFacingCameraFrame.getX(), tagFacingCameraFrame.getY(), tagFacingCameraFrame.getZ());
        R.put(0, 0, rotationVector[0], rotationVector[1], rotationVector[2]);

        MatOfPoint2f imagePointsBottom = new MatOfPoint2f();
        Calib3d.projectPoints(bottom, R, T, K, distortionCoeffs, imagePointsBottom);

        MatOfPoint2f imagePointsTop = new MatOfPoint2f();
        Calib3d.projectPoints(top, R, T, K, distortionCoeffs, imagePointsTop);
        
        ArrayList<Point> topCornerPoints = new ArrayList<Point>();

        // draw from bottom points to top points - pillars
        for(int i = 0; i < 4; i++)
        {
            var x1 = imagePointsBottom.get(i, 0)[0];
            var y1 = imagePointsBottom.get(i, 0)[1];
            var x2 = imagePointsTop.get(i, 0)[0];
            var y2 = imagePointsTop.get(i, 0)[1];

            topCornerPoints.add(new Point(x2, y2));

            Imgproc.line(mat,
                new Point(x1, y1),
                new Point(x2, y2),
                outlineColor,
                2);
        }

        MatOfPoint topCornersTemp = new MatOfPoint();
        topCornersTemp.fromList(topCornerPoints);
        ArrayList<MatOfPoint> topCorners = new ArrayList<>();
        topCorners.add(topCornersTemp);

        Imgproc.polylines(mat, topCorners, true, outlineColor, 2);
      } /* end draw a frustum in front of the AprilTag */

      /* 
      This Transform3d from tagFacingCameraFrame to tagInCameraFrame is required for the correct robot pose.
      It appears to arise from the tag facing the camera thus Pi radians rotated or CCW/CW flipped from
      the mathematically described pose from the estimator. The true rotation has to be used to get the
      right robot pose. It seems that the T and R from the estimator could take care of all this (it is
      consistent without the extra transform when drawing the tag and orientation box).

      From PhotonVision this is likely the explanation:
      * The AprilTag pose rotation outputs are X left, Y down, Z away from the tag
      * with the tag facing
      * the camera upright and the camera facing the target parallel to the floor.
      * But our OpenCV
      * solvePNP code would have X left, Y up, Z towards the camera with the target
      * facing the camera
      * and both parallel to the floor. So we apply a base rotation to the rotation
      * component of the
      * apriltag pose to make it consistent with the EDN system that OpenCV uses,
      * internally a 180
      * rotation about the X axis
      */
      var tagInCameraFrame = new Transform3d(
      new Translation3d(
                    tagFacingCameraFrame.getX(),
                    tagFacingCameraFrame.getY(),
                    tagFacingCameraFrame.getZ()),
      new Rotation3d(
                  -tagFacingCameraFrame.getRotation().getX() - Math.PI,
                  -tagFacingCameraFrame.getRotation().getY(),
                  tagFacingCameraFrame.getRotation().getZ() - Math.PI));
      /*
      from WPILib documentation Drive classes:
      Axis Conventions:
      The drive classes use the NWU axes convention (North-West-Up as external reference in the world frame).
      The positive X axis points ahead, the positive Y axis points left, and the positive Z axis points up.
      We use NWU here because the rest of the library, and math in general, use NWU axes convention.

      Joysticks follow NED (North-East-Down) convention, where the positive X axis points ahead, the
      positive Y axis points right, and the positive Z axis points down. However, it’s important to note
      that axes values are rotations around the respective axes, not translations. When viewed with each
      axis pointing toward you, CCW is a positive value and CW is a negative value. Pushing forward on the joystick is a CW rotation around the Y axis, so you get a negative value. Pushing to the right is a CCW rotation around the X axis, so you get a positive value.
      */
      // OpenCV and WPILib estimator layout of axes is EDN and field WPILib is NWU;
      // need x -> -y , y -> -z , z -> x and same for differential rotations
      tagInCameraFrame = CoordinateSystem.convert(tagInCameraFrame, CoordinateSystem.EDN(), CoordinateSystem.NWU());
      // // WPILib CoordinateSystem.convert was wrong for transforms in 2023 so this patch was used
      // { // corrected convert
      // var from = CoordinateSystem.EDN();
      // var to = CoordinateSystem.NWU();
      // tagInCameraFrame = new Transform3d(
      //           CoordinateSystem.convert(tagInCameraFrame.getTranslation(), from, to),
      //           CoordinateSystem.convert(new Rotation3d(), to, from)
      //               .plus(CoordinateSystem.convert(tagInCameraFrame.getRotation(), from, to)));
      // } // end of corrected convert
      
      var // transform to camera from robot chassis center at floor level - robot specific!
      cameraInRobotFrame = new Transform3d(       
            new Translation3d(0., 0., 0.), // camera at center bottom of robot
            // new Translation3d(0.2, 0., 0.8), // camera in front of center of robot and above ground
            // new Rotation3d(0.0, Units.degreesToRadians(0.), Units.degreesToRadians(0.0)) // camera in line with robot chassis
            new Rotation3d(0., Units.degreesToRadians(-25.), 0.) // camera in line with robot chassis, pointing up slightly
            );
      // x + roll is camera rolling CCW relative to the robot looking facing the robot
      // y + pitch is camera pointing down relative to the robot. -25 camera points up; +25 points down; sign is correct but backwards of LL
      // z + yaw is camera pointing to the left of robot looking down on it (CCW relative to the robot)

      var // robot in field is the composite of 3 pieces
      robotInFieldFrame = ComputerVisionUtil.objectToRobotPose(tagInFieldFrame,  tagInCameraFrame,  cameraInRobotFrame);

      // the above transforms match LimeLight Vision botpose_wpiblue network tables entries
      // as they display in AdvantageScope 3D Field robotInFieldFrame

      // end transforms to get the robot pose from this vision tag pose

      // put detection and pose information on dashboards
      Rotation3d rot = robotInFieldFrame.getRotation();

      // arrays don't display on the SmartDashboard; they display on the SmartDashboard tab of ShuffleBoard
      SmartDashboard.putNumberArray("robotPose3d_" + detection.getId(),new double[] {
                      robotInFieldFrame.getX(), robotInFieldFrame.getY(), robotInFieldFrame.getZ(),
                      rot.getX(), rot.getY(), rot.getZ()} );
      
      SmartDashboard.putNumber("detectionDecisionMargin_" + detection.getId(), detection.getDecisionMargin());

      // put out to NetworkTables tag and robot pose for this tag in AdvantageScope format
      publishRobotPose.get(detection.getId()).set(robotInFieldFrame);
      publishTagPose.get(detection.getId()).set(tagInFieldFrame);
    } // end of all detections

    long frameEndTime = System.nanoTime();
    latency = (int)(1.e-6*(frameEndTime - acquisitionTime.acquisitionTime)); // ms
    SmartDashboard.putNumber("latency", latency);

    // put list of tags onto dashboard
    pubTags.set(tags.stream().mapToLong(Long::longValue).toArray());
  
    // all the data available at this point.

    // Give the output stream a new image to display
    outputStream.putFrame(mat);
  }
    pubTags.close();
  }

  /**
   * image from camera in OpenCV Mat form and detections
   */
  public class Image
  {
      private Mat mat = new Mat();
      private AprilTagDetection[] detections;
      private long acquisitionTime = 0L;
      private int frameNumber = 0;
      private boolean isFreshImage = false;

      public synchronized void setImage(Mat mat, AprilTagDetection[] detections, long acquisitionTime, int frameNumber)
      {
          mat.copyTo(this.mat);
          this.detections = detections.clone();
          this.acquisitionTime = acquisitionTime;
          this.frameNumber = frameNumber;
          this.isFreshImage = true;
          notify(); // fresh image so tell whoever is waiting for it
      }

      public synchronized AprilTagDetection[] getImage(Mat mat,  AcquisitionTime acquisitionTime)
      {
          try
          {
              while(!this.isFreshImage) // make sure awakened for the right reason
              {
                  wait(0L, 0); // stale image so wait for a new image no timeout
              }
          } 
          catch (Exception e)
          {
              System.out.println("getImage Exception " + e.toString());
              throw new RuntimeException(e);
          }
          this.isFreshImage = false;
          this.mat.copyTo(mat);
          acquisitionTime.acquisitionTime = this.acquisitionTime;
          acquisitionTime.frameNumber = this.frameNumber;
          return this.detections;
      }

      public synchronized boolean isFreshImage()
      {
          return this.isFreshImage;
      }
  }

  class AcquisitionTime
  {
    protected int frameNumber = 0;
    protected long acquisitionTime = 0;
  }
}
/*
WPILib:
16 Tags on file
AprilTag(ID: 1, pose: Pose3d(Translation3d(X: 15.08, Y: 0.25, Z: 1.36), Rotation3d(Quaternion(0.5000000000000001, 0.0, 0.0, 0.8660254037844386)))) 0.0, 0.0, 120.0 [degrees]
AprilTag(ID: 2, pose: Pose3d(Translation3d(X: 16.19, Y: 0.88, Z: 1.36), Rotation3d(Quaternion(0.5000000000000001, 0.0, 0.0, 0.8660254037844386)))) 0.0, 0.0, 120.0 [degrees]
AprilTag(ID: 3, pose: Pose3d(Translation3d(X: 16.58, Y: 4.98, Z: 1.45), Rotation3d(Quaternion(6.123233995736766E-17, 0.0, 0.0, 1.0)))) 0.0, 0.0, 180.0 [degrees]
AprilTag(ID: 4, pose: Pose3d(Translation3d(X: 16.58, Y: 5.55, Z: 1.45), Rotation3d(Quaternion(6.123233995736766E-17, 0.0, 0.0, 1.0)))) 0.0, 0.0, 180.0 [degrees]
AprilTag(ID: 5, pose: Pose3d(Translation3d(X: 14.70, Y: 8.20, Z: 1.36), Rotation3d(Quaternion(-0.7071067811865475, -0.0, 0.0, 0.7071067811865476)))) 0.0, 0.0, -90.0 [degrees]
AprilTag(ID: 6, pose: Pose3d(Translation3d(X: 1.84, Y: 8.20, Z: 1.36), Rotation3d(Quaternion(-0.7071067811865475, -0.0, 0.0, 0.7071067811865476)))) 0.0, 0.0, -90.0 [degrees]
AprilTag(ID: 7, pose: Pose3d(Translation3d(X: -0.04, Y: 5.55, Z: 1.45), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0)))) 0.0, 0.0, 0.0 [degrees]
AprilTag(ID: 8, pose: Pose3d(Translation3d(X: -0.04, Y: 4.98, Z: 1.45), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0)))) 0.0, 0.0, 0.0 [degrees]
AprilTag(ID: 9, pose: Pose3d(Translation3d(X: 0.36, Y: 0.88, Z: 1.36), Rotation3d(Quaternion(0.8660254037844387, 0.0, 0.0, 0.49999999999999994)))) 0.0, 0.0, 60.0 [degrees]
AprilTag(ID: 10, pose: Pose3d(Translation3d(X: 1.46, Y: 0.25, Z: 1.36), Rotation3d(Quaternion(0.8660254037844387, 0.0, 0.0, 0.49999999999999994)))) 0.0, 0.0, 60.0 [degrees]
AprilTag(ID: 11, pose: Pose3d(Translation3d(X: 11.90, Y: 3.71, Z: 1.32), Rotation3d(Quaternion(-0.8660254037844387, -0.0, 0.0, 0.49999999999999994)))) 0.0, 0.0, -60.0 [degrees]
AprilTag(ID: 12, pose: Pose3d(Translation3d(X: 11.90, Y: 4.50, Z: 1.32), Rotation3d(Quaternion(0.8660254037844387, 0.0, 0.0, 0.49999999999999994)))) 0.0, 0.0, 60.0 [degrees]
AprilTag(ID: 13, pose: Pose3d(Translation3d(X: 11.22, Y: 4.11, Z: 1.32), Rotation3d(Quaternion(6.123233995736766E-17, 0.0, 0.0, 1.0)))) 0.0, 0.0, 180.0 [degrees]
AprilTag(ID: 14, pose: Pose3d(Translation3d(X: 5.32, Y: 4.11, Z: 1.32), Rotation3d(Quaternion(1.0, 0.0, 0.0, 0.0)))) 0.0, 0.0, 0.0 [degrees]
AprilTag(ID: 15, pose: Pose3d(Translation3d(X: 4.64, Y: 4.50, Z: 1.32), Rotation3d(Quaternion(0.5000000000000001, 0.0, 0.0, 0.8660254037844386)))) 0.0, 0.0, 120.0 [degrees]
AprilTag(ID: 16, pose: Pose3d(Translation3d(X: 4.64, Y: 3.71, Z: 1.32), Rotation3d(Quaternion(-0.49999999999999983, -0.0, 0.0, 0.8660254037844388)))) 0.0, 0.0, -120.0 [degrees]

LimeLight fmap at
https://docs.limelightvision.io/docs/docs-limelight/pipeline-apriltag/apriltag-map-specification

*/

/* from photonvision
    * Calculates the horizontal and vertical FOV components from a given diagonal FOV and image size.
    *
    * @param diagonalFoV Diagonal FOV in degrees
    * @param imageWidth Image width in pixels
    * @param imageHeight Image height in pixels
    * @return Horizontal and vertical FOV in degrees

  public static DoubleCouple calculateHorizontalVerticalFoV(
          double diagonalFoV, int imageWidth, int imageHeight) {
      diagonalFoV = Math.toRadians(diagonalFoV);
      double diagonalAspect = Math.hypot(imageWidth, imageHeight);

      double horizontalView =
              Math.atan(Math.tan(diagonalFoV / 2) * (imageWidth / diagonalAspect)) * 2;
      double verticalView = Math.atan(Math.tan(diagonalFoV / 2) * (imageHeight / diagonalAspect)) * 2;

      return new DoubleCouple(Math.toDegrees(horizontalView), Math.toDegrees(verticalView));
  }

*/