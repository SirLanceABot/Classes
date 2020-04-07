package contoursUtils;

import org.opencv.core.Mat;

public class contours {

    // essentially a copy of OpenCV matchShapes without calculating the Moments
    // which then must be passed into this method.

    // Allows efficiency of not recalculating the target shape moments every time.
    // Does all 3 comparisons since it is very little more than to do one.

    // Additional comparisons are cosine (theta) angle between two 7-dimension
    // vectors and normalized two ways - maximum value of each pair of elements
    // and total value of the pair

    // Moments are scale and translation invariant and Hu added 6 moments I1 - I6 to
    // be rotation invariant.

    // The last Hu moment, I7, is skew invariant, which enables it to distinguish,
    // with the flip in sign, mirror images of otherwise identical images.

    // Mirror images will report as slightly different because of the I7. It seems
    // to me a better strategy to detect mirror images would be to verify I1-I6 are
    // very similar then check independently the I7 for similarity except for a sign
    // flip otherwise these two images will be reported as very similar differing
    // only because of the I7 differences globbed into the small I1 - I6
    // contributions.

    // https://homepages.inf.ed.ac.uk/rbf/CVonline/LOCAL_COPIES/SHUTLER3/node8.html
    // https://en.wikipedia.org/wiki/Image_moment
    // https://www.learnopencv.com/shape-matching-using-hu-moments-c-python/ [note
    // error in equation (10) method 3 formula]
    // http://zoi.utia.cas.cz/files/chapter_2D_rotation.pdf
    // https://www.umass.edu/landeco/teaching/multivariate/readings/McCune.and.Grace.2002.chapter9.pdf
    // [Data Transformations]

    public static double[] compareShapes(Mat HuShape1, Mat HuShape2) {
        if ((HuShape1.rows() != 7) || (HuShape2.rows() != 7))
            throw new IllegalArgumentException("HuMoments must be 7x1");
        if ((HuShape1.cols() != 1) || (HuShape2.cols() != 1))
            throw new IllegalArgumentException("HuMoments must be 7x1");

        double[] compare = { 0., 0., 0., 0., 0., 0., 0. };
        boolean anyA = false, anyB = false;
        double[] HuTemp1 = new double[7], HuTemp2 = new double[7];
        double eps = 1.e-20;
        HuShape1.get(0, 0, HuTemp1);
        HuShape2.get(0, 0, HuTemp2);

        double sumABm4 = 0., sumA2m4 = 0., sumB2m4 = 0.; // method 4
        double sumABm5 = 0., sumA2m5 = 0., sumB2m5 = 0.; // method 5

        for (int idx = 0; idx < 7; idx++) {
            if (HuTemp1[idx] != 0.)
                anyA = true;
            if (HuTemp2[idx] != 0.)
                anyB = true;
            if ((Math.abs(HuTemp1[idx]) > eps) && (Math.abs(HuTemp2[idx]) > eps)) {
                double mA = Math.copySign(Math.log10(Math.abs(HuTemp1[idx])), HuTemp1[idx]); // typically seen with
                                                                                             // minus sign in front
                double mB = Math.copySign(Math.log10(Math.abs(HuTemp2[idx])), HuTemp2[idx]); // but don't bother since
                                                                                             // it's abs here in 3
                                                                                             // compares
                compare[0] += Math.abs((1. / mA) - (1. / mB));
                compare[1] += Math.abs(mA - mB);
                compare[2] = Math.max(compare[2], Math.abs((mA - mB) / mA));
                // additions to the OpenCV matchShapres start here
                // normalize to max absolute value
                mA = HuTemp1[idx] / Math.max(Math.abs(HuTemp1[idx]), Math.abs(HuTemp2[idx]));
                mB = HuTemp2[idx] / Math.max(Math.abs(HuTemp1[idx]), Math.abs(HuTemp2[idx]));
                sumABm4 += mA * mB; // summations for the cosine (theta) calculation
                sumA2m4 += mA * mA;
                sumB2m4 += mB * mB;
                compare[5] += Math.abs(mA - mB); // compare difference like I2 but normalized differently
                // normalize to unit length
                mA = HuTemp1[idx] / Math.sqrt(HuTemp1[idx] * HuTemp1[idx] + HuTemp2[idx] * HuTemp2[idx]);
                mB = HuTemp2[idx] / Math.sqrt(HuTemp1[idx] * HuTemp1[idx] + HuTemp2[idx] * HuTemp2[idx]);
                sumABm5 += mA * mB; // summations for the cosine (theta) calculation
                sumA2m5 += mA * mA;
                sumB2m5 += mB * mB;
                compare[6] += Math.abs(mA - mB); // compare differences like I2 but normalized differently
            }
        }
        // cosine (theta) - the angle between two 7-dimensional vectors
        // sum of cross products divided by product of the square roots of the sum of the squares
        compare[3] = sumABm4 / (Math.sqrt(sumA2m4) * Math.sqrt(sumB2m4));
        compare[4] = sumABm5 / (Math.sqrt(sumA2m5) * Math.sqrt(sumB2m5));

        // couldn't make the calculation so bailout with max differences indicated
        if (anyA != anyB) {
            compare[0] = Double.MAX_VALUE;
            compare[1] = Double.MAX_VALUE;
            compare[2] = Double.MAX_VALUE;
            compare[3] = -1.;
            compare[4] = -1.;
            compare[5] = Double.MAX_VALUE;
            compare[6] = Double.MAX_VALUE;
        }

        return compare;
    }
}