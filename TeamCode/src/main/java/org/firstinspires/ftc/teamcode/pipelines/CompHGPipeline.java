package org.firstinspires.ftc.teamcode.pipelines;


public class CompHGPipeline extends UGAngleHighGoalPipeline {
    private UGBasicHighGoalPipeline.Mode color;
    public CompHGPipeline(UGBasicHighGoalPipeline.Mode color) {
        super(55, color);
        this.color = color;
    }

    public double getTargetAngle() {
        return calculateYaw(color);
    }
    public double getTargetPitch() {
        return calculatePitch(color);
    }
    public boolean isTargetVisible() {
        if (color == UGBasicHighGoalPipeline.Mode.RED_ONLY)
            return isRedVisible();
        return isBlueVisible();
    }


}
