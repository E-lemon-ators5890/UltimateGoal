package org.firstinspires.ftc.teamcode.pipelines;


public class CompHGPipeline extends UGDistanceHighGoalPipeline {
    private UGBasicHighGoalPipeline.Mode color;
    public CompHGPipeline(UGBasicHighGoalPipeline.Mode color) {
        super(color);
        this.color = color;
    }

    public double getTargetAngle() {
        return calculateYaw(color);
    }
    public double getTargetPitch() {
        return calculatePitch(color);
    }

    public boolean isTargetVisible() {
        return isTargetVisible(color);
    }

}
