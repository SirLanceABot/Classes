package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

public class Circles {

    public static Mat drawSiemensStar() {
        int radius = 350;
        int margin = 10;
        Mat star = Mat.zeros(2*radius+margin, 2*radius+margin, CvType.CV_8UC1);
        int xCenter = star.cols()/2;
        int yCenter = star.rows()/2;
        for (double angle = 0.; angle < 2.*Math.PI; angle += 2.*Math.PI/360.*60.) {
            int x1Circumference = (int)(radius*Math.sin(angle)) + xCenter;
            int y1Circumference = (int)(radius*Math.cos(angle)) + yCenter;
            int x2Circumference = (int)(radius*Math.sin(angle+2.*Math.PI/360.*5.)) + xCenter;
            int y2Circumference = (int)(radius*Math.cos(angle+2.*Math.PI/360.*5.)) + yCenter;
            MatOfPoint triangle = new MatOfPoint(new Point(xCenter, yCenter),
                    new Point(x1Circumference, y1Circumference), new Point(x2Circumference, y2Circumference));
            List<MatOfPoint> triangles = new ArrayList<>();
            triangles.add(triangle);
            Imgproc.polylines(star, triangles, true, new Scalar(210),1,Imgproc.LINE_AA);
            Imgproc.fillPoly(star, triangles, new Scalar(210), Imgproc.LINE_AA);
            Imgproc.circle(star, new Point(xCenter, yCenter), radius, new Scalar(255), 4, Imgproc.LINE_AA);
            Imgproc.circle(star, new Point(xCenter, yCenter), radius-4, new Scalar(0), 4, Imgproc.LINE_AA);
        }
        return star;
    }
}
