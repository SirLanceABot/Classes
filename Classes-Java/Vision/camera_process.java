package org.usfirst.frc.team4237.robot;

import java.util.ArrayList;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Rect;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;


class camera_process implements Runnable {

	private static Object COGlock = new Object();
	private int lCOGX;
	private int lCOGY;
	public boolean lCOGFRESH;

	class targetinfo
	{
		public int COGX;
		public int COGY;
		private boolean COGFRESH;

		public targetinfo()
		{
			synchronized(COGlock)
			{
				COGX = lCOGX;
				COGY = lCOGY;
				COGFRESH = lCOGFRESH;
				lCOGFRESH = false;
			}
		}

		boolean isFresh()
		{
			boolean lIsFresh = COGFRESH;
			COGFRESH = false;
			return lIsFresh;
		}

		public String toString()
		{
			return COGX + " " + COGY + " " + COGFRESH;
		}
	}

	public void run() {visionPipeline();}

	private void visionPipeline()
	{
		synchronized(COGlock)
		{
			lCOGX = -1;
			lCOGY = -1;
			lCOGFRESH = false;
		}

		// Get the UsbCamera from CameraServer
		UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
		// Set the resolution
		camera.setResolution(320, 240);

		// Get a CvSink. This will capture Mats from the camera
		CvSink cvSink = CameraServer.getInstance().getVideo();
		// Setup a CvSource. This will send images back to the Dashboard
		CvSource outputStream = CameraServer.getInstance().putVideo("Rectangle", 320, 240);

		// Mats are very memory expensive. Lets reuse this Mat.
		Mat mat = new Mat();

		GripPipelineBasement gripper = new GripPipelineBasement();

		while (true)
		{
			// This lets the robot stop this thread when restarting robot code or
			// deploying.  Maybe.  Is it still true?
			if(Thread.interrupted())
			{ // I've been told to interrupt so go to sleep
				// debug output
				System.out.println("[Vision] thread interrupted - go to sleep");

				synchronized(COGlock) // one last (bad) target info before sleeping
				{
					lCOGX = -1;
					lCOGY = -1;
					lCOGFRESH = true;
				}

				synchronized(Constants.LOCKS.cameraThread)
				{
					while(true)
					{
						try {
							Constants.LOCKS.cameraThread.wait(); // go to sleep until notified
							break; // notify() comes here so awaken by leaving the sleep loop
						} catch (InterruptedException e) {  // thread interrupt() comes here so go back to sleep by continuing the sleep loop
							continue;
						}
					}
				}
			}

			// Tell the CvSink to grab a frame from the camera and put it
			// in the source mat.  If there is an error notify the output.
			if (cvSink.grabFrame(mat) == 0) {
				// Send the output the error.
				outputStream.notifyError(cvSink.getError());
				// skip the rest of the current iteration
				continue;
			}

			gripper.process(mat);

			ArrayList<MatOfPoint> contoursFiltered = new ArrayList<MatOfPoint>(gripper.filterContoursOutput());

			if (contoursFiltered.isEmpty())
			{ // no contours in this frame
				// debug output
				System.out.println("[Vision] No Contours");
				Imgproc.putText(mat, "No Contours", new Point(50, 22), Core.FONT_HERSHEY_SIMPLEX, 1., new Scalar(255,255,255), 2);
				synchronized(COGlock)
				{
					lCOGX = -1;
					lCOGY = -1;
					lCOGFRESH = true;
				}
			}
			else
			{ // process the contours
				// debug output
				System.out.println("[Vision] " + contoursFiltered.size() + " contours");

				// draw all contours at once (negative index); could draw one at a time within the contour loop but this has to be more efficient
				Imgproc.drawContours(mat, contoursFiltered, -1, new Scalar(255,255,255), 2); // + thickness => empty ; - thickness => filled

				for(MatOfPoint aContour : contoursFiltered) // iterate one contour at a time through all the contours
				{
					// debug output
					System.out.println("[Vision] " + aContour.size() + " points in contour"); // a contour is a bunch of points - how many?

					for(Point aPoint : aContour.toArray()) // convert MatofPoint to an array of those Points and iterate (could do list of Points but no need for this)
					{System.out.print("[Vision] " + aPoint + " ");} // print each point

					Rect br = Imgproc.boundingRect(aContour); // bounding upright rectangle for the contour's points
					int cogX = br.x + (br.width/2); // center of gravity
					int cogY = br.y + (br.height/2);
					synchronized(COGlock)
					{
						lCOGX = cogX;
						lCOGY = cogY;
						lCOGFRESH = true;
					}

					// draw center of gravity markers
					Imgproc.drawMarker(mat, new Point(cogX,cogY), new Scalar(190, 190, 190), Imgproc.MARKER_CROSS , 9 , 2 , 1);
					// debug output
					System.out.printf("[Vision] %d %d %d %d %d %d\n", br.x, br.y, br.width, br.height, cogX, cogY);
				}
				// an example drawing function: Imgproc.rectangle(mat, new Point(100, 100), new Point(200, 200), new Scalar(255, 255, 255), 5);
			}

			// Give the output stream a new image to display
			outputStream.putFrame(mat);
			//System.out.println(System.currentTimeMillis());
			//			try {
			//				Thread.sleep(1000); // long sleep for testing purposes
			//			} catch (InterruptedException e) {
			//				e.printStackTrace();
			//			}
		}
	}
}
