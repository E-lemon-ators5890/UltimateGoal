package org.firstinspires.ftc.teamcode.pipelines;

import android.util.Pair;

import org.opencv.core.*;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.*;
import java.util.stream.Collectors;
public class UGBasicHighGoalPipeline extends OpenCvPipeline {
    // Thresholds for filtering red and blue contours such as High Goal
    public static double MIN_COLOR_THRESHOLD = 155;
    public static double MAX_COLOR_THRESHOLD = 256;

    // Color to target/process
    public enum Mode {
        RED_ONLY, BLUE_ONLY, BOTH
    }

    protected double centerX;
    protected double centerY;

    // YCbCr Channel numbers (constants)
    // Wikipedia article: https://en.wikipedia.org/wiki/YCbCr
    private final int YCBCR_Y_CHANNEL = 0; // grayscale
    private final int YCBCR_CB_CHANNEL = 1; // Blue-chroma difference
    private final int YCBCR_CR_CHANNEL = 2; // Red-chroma difference

    private Mode currentMode;

    // YCrCb color space to extract red, blue, and grayscale easier
    private Mat adjustedColorSpace;
    // Channel and Threshold mat (for tracking)
    private Mat currentChannel, currentThreshold;

    // List to read contours into
    private List<MatOfPoint> currentContours, interiorContours;
    private Point blueCenter, redCenter;
    private int xOffset = 0;

    public UGBasicHighGoalPipeline() {
        this(Mode.RED_ONLY);
    }

    public UGBasicHighGoalPipeline(Mode mode) {
        this.currentMode = mode;

        adjustedColorSpace = new Mat();

        currentChannel = new Mat();
        currentThreshold = new Mat();

        currentContours = new ArrayList<>(3);
        interiorContours = new ArrayList<>(3);

        blueCenter = new Point();
        redCenter = new Point();


    }

    @Override
    public void init(Mat mat) {
        super.init(mat);
        centerX = ((double) mat.width() / 2) - 0.5;
        centerY = ((double) mat.height() / 2) - 0.5;
    }

    @Override
    public Mat processFrame(Mat input) {
        Imgproc.cvtColor(input, adjustedColorSpace, Imgproc.COLOR_RGB2YCrCb);

        if(currentMode == Mode.RED_ONLY || currentMode == Mode.BOTH)
        {
            // Extract necessary Channels to apropritae Mats
            Core.extractChannel(adjustedColorSpace, currentChannel, YCBCR_CB_CHANNEL);
            redCenter = findTarget(input, currentChannel);
            if (redCenter != null) {
                Imgproc.drawMarker(input, redCenter, new Scalar (255, 0, 0));
            }
        }

        if(currentMode == Mode.BLUE_ONLY || currentMode == Mode.BOTH)
        {
            MIN_COLOR_THRESHOLD = 142;
            // Extract necessary Channels to apropritae Mats
            Core.extractChannel(adjustedColorSpace, currentChannel, YCBCR_CR_CHANNEL);
            blueCenter = findTarget(input, currentChannel);
            if (blueCenter != null) {
                Imgproc.drawMarker(input, blueCenter, new Scalar (255, 0, 0));
            }
        }
        // Convert RGB input to YCrCB, this is for better Red and blue identification

        currentThreshold.release();
        currentChannel.release();

        adjustedColorSpace.release();
        return input;

    }

    public Point getCenterRed() {
        return redCenter;
    }

    public Point getCenterBlue() {
        return blueCenter;
    }
    /**
     * @return whether the red high goal is visible
     */
    public boolean isRedVisible() {
        return (redCenter != null);
    }

    /**
     * @return whether the blue high goal is visible
     */
    public boolean isBlueVisible() {
        return (blueCenter != null);
    }

    /**
     *
     * @param channel The Cr or Cb channel to filter out red & blue high goal
     * @return High goal target
     */
    private Point findTarget(Mat draw, Mat channel) {
        // Clears current contour lists
        currentContours.clear();
        interiorContours.clear();

        // Filters channel based on threshold - giving us a binary image
        Imgproc.threshold(channel, currentThreshold, MIN_COLOR_THRESHOLD, MAX_COLOR_THRESHOLD, Imgproc.THRESH_BINARY);
        int kernelSize = 3;
        Mat element = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size( kernelSize + 1,  kernelSize + 1),
                new Point(kernelSize, kernelSize));
        if (currentMode == Mode.BLUE_ONLY) {
            kernelSize = 6;
            element = Imgproc.getStructuringElement(Imgproc.CV_SHAPE_RECT, new Size( kernelSize + 1,  kernelSize + 1),
                    new Point(kernelSize, kernelSize));
        }
        Imgproc.dilate(currentThreshold, currentThreshold, element);

        // Finding the contours along with its heirarchy (so we can find interior/child contours)
        Mat hierarchy = new Mat();
        Imgproc.findContours(currentThreshold, currentContours, hierarchy, Imgproc.RETR_CCOMP, Imgproc.CHAIN_APPROX_NONE);

        // Creating new list of contours to add suitable high goal candidates (ones that have a child and fit aspect ratio)
        List<Pair> filteredContours = new ArrayList<>();
        Imgproc.drawContours(currentChannel, currentContours, -1, new Scalar(255, 0, 0));
        for (int i = 0; i < currentContours.size(); i++) {
            //Filtering out all countours that don't have children.
            Imgproc.drawContours(adjustedColorSpace, currentContours, i, new Scalar(0, 0, 0), 2);
            if(hierarchy.get(0, i)[2] != -1) {
                MatOfPoint currentContour = currentContours.get(i);
                MatOfPoint childContour = currentContours.get((int) hierarchy.get(0, i)[2]);
                //Further filtering out contours that don't pass this apsect ratio.
                if (isPossibleContour(currentContour) && Imgproc.contourArea(childContour) > 25) {
                    filteredContours.add(new Pair(currentContour, childContour));
                }
            }
        }

        if (filteredContours.isEmpty()) {
            return null;
        } else {

            // Comparing width instead of area because wobble goals that are close to the camera tend to have a large area
            Pair biggestContourPair = Collections.max(filteredContours, Comparator.comparingDouble(t0 -> Imgproc.minAreaRect(new MatOfPoint2f(((MatOfPoint) t0.first).toArray())).size.area()));

            Moments targetMoments = Imgproc.moments(((MatOfPoint) biggestContourPair.second));
            Point targetCenter = new Point((int) (targetMoments.m10 / targetMoments.m00) + xOffset, (int) (targetMoments.m01 / targetMoments.m00));


            Rect boundingRect = Imgproc.boundingRect(((MatOfPoint) biggestContourPair.first));
            Imgproc.rectangle(draw, boundingRect, new Scalar(255, 0, 255), 3);

            return targetCenter;
        }
    }

    /**
     * @param contour to be checked
     * @return whether the contour fits high goal shape
     */
    private boolean isPossibleContour(MatOfPoint contour) {
        Rect boundingRect = Imgproc.boundingRect(contour);
        double aspectRatio = (double) boundingRect.width / (double) boundingRect.height;
        Imgproc.putText(adjustedColorSpace, Double.toString(aspectRatio), new Point(boundingRect.x, boundingRect.y), 2, 1, new Scalar(255, 255, 255));
        return Imgproc.contourArea(contour) > 50  && aspectRatio > 0.55 && aspectRatio < 1.85;
    }

    public Point getCenterofRect(Rect rect) {
        if (rect == null) {
            return new Point(centerX, centerY);
        }
        return new Point(rect.x + rect.width / 2.0, rect.y + rect.height / 2.0);
    }

    public void setXOffset(int offset) {
        this.xOffset = offset;
    }
    public int getxOffset() {
        return xOffset;
    }
}


