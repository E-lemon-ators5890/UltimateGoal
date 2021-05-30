package org.firstinspires.ftc.teamcode.pipelines;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;

public class UGAngleHighGoalPipeline extends UGBasicHighGoalPipeline {

    class Fraction {
        private int numerator, denominator;

        Fraction(long a, long b) {
            numerator = (int) (a / gcd(a, b));
            denominator = (int) (b / gcd(a, b));
        }

        /**
         * @return the greatest common denominator
         */
        private long gcd(long a, long b) {
            return b == 0 ? a : gcd(b, a % b);
        }

        public int getNumerator() {
            return numerator;
        }

        public int getDenominator() {
            return denominator;
        }
    }

    // Camera Settings
    protected int imageWidth;
    protected int imageHeight;

    private double fov;
    private double horizontalFocalLength;
    private double verticalFocalLength;

    public UGAngleHighGoalPipeline() {
        this(55);
    }
    public UGAngleHighGoalPipeline(double fov, Mode mode) {
        super(mode);
        this.fov = fov;
    }

    public UGAngleHighGoalPipeline(double fov) {
        this(fov, Mode.BLUE_ONLY);
    }

    @Override
    public void init(Mat mat) {
        super.init(mat);

        imageWidth = mat.width();
        imageHeight = mat.height();

        // pinhole model calculations
        double diagonalView = Math.toRadians(this.fov);
        Fraction aspectFraction = new Fraction(this.imageWidth, this.imageHeight);
        int horizontalRatio = aspectFraction.getNumerator();
        int verticalRatio = aspectFraction.getDenominator();
        double diagonalAspect = Math.hypot(horizontalRatio, verticalRatio);
        double horizontalView = Math.atan(Math.tan(diagonalView / 2) * (horizontalRatio / diagonalAspect)) * 2;
        double verticalView = Math.atan(Math.tan(diagonalView / 2) * (verticalRatio / diagonalAspect)) * 2;
        horizontalFocalLength = this.imageWidth / (2 * Math.tan(horizontalView / 2));
        verticalFocalLength = this.imageHeight / (2 * Math.tan(verticalView / 2));
    }

    @Override
    public Mat processFrame(Mat input) {

       super.processFrame(input);
       return input;
    }

    /**
     * @param color         Alliance color
     */
    public double calculateYaw(Mode color) {
        Point currentPoint = color == Mode.RED_ONLY ? getCenterRed() : getCenterBlue();
        if (currentPoint == null)
            return 0;
        double targetCenterX = currentPoint.x;
        return Math.toDegrees(
                Math.atan((targetCenterX - centerX) / horizontalFocalLength)
        );
    }

    /**
     * @param color         Alliance color
     */
    public double calculatePitch(Mode color) {
        Point currentPoint = color == Mode.RED_ONLY ? getCenterRed() : getCenterBlue();
        if (currentPoint == null)
            return 0;

        double targetCenterY = currentPoint.y;
        return -Math.toDegrees(
                Math.atan((targetCenterY - centerY) / verticalFocalLength)
        );
    }

}

