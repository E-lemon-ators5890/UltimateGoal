package org.firstinspires.ftc.teamcode.pipelines;

import org.opencv.core.Mat;
import org.opencv.core.Point;


public class UGDistanceHighGoalPipeline extends UGAngleHighGoalPipeline {
    private double cameraAngle, deltaHeight;
    private double shooterAngle = Math.toRadians(29);
    Mode mode;
    public UGDistanceHighGoalPipeline(Mode mode) {
        this(55, mode, 29, 43, 10.75);
    }
    public UGDistanceHighGoalPipeline(double fov, Mode mode, double cameraAngle, double hgHeight, double cameraHeight) {
        super(fov, mode);
        this.cameraAngle = cameraAngle;
        deltaHeight = hgHeight - cameraHeight;
        this.mode = mode;
    }

    @Override
    public void init(Mat mat) {
        super.init(mat);
    }

    @Override
    public Mat processFrame(Mat input) {

        input = super.processFrame(input);
        return input;
    }

    @Override
    public double calculatePitch(Mode color) {
        return cameraAngle + super.calculatePitch(mode);
    }

    public double getDistance() {
        if (!isTargetVisible(mode))
            return -1;
        return (deltaHeight / Math.tan(Math.toRadians(calculatePitch(mode))));
    }

    public boolean isTargetVisible(Mode color) {
        if (color == UGBasicHighGoalPipeline.Mode.RED_ONLY)
            return isRedVisible();
        return isBlueVisible();
    }


}

