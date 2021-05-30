package org.firstinspires.ftc.teamcode.pipelines;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.vision.UGRectRingPipeline;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
@Config
public class RingPipelineEx extends UGRectRingPipeline {

    public static double MIN = 105;
    public static double MAX = 110;

    public void setBottomRectangle(double x, double y) {
        setBottomRectHeightPercentage(y);
        setBottomRectWidthPercentage(x);
    }
    public void setTopRectangle(double x, double y) {
        setTopRectHeightPercentage(y);
        setTopRectWidthPercentage(x);
    }
    public void setRectangleSize(int w, int h) {
        setRectangleHeight(h);
        setRectangleWidth(w);
    }

    public Stack getStack() {
        if (Math.abs(getTopAverage() - getBottomAverage()) < getThreshold() && (getTopAverage() <= MAX && getBottomAverage() <= MAX)) {
            return Stack.FOUR;
        } else if (Math.abs(getTopAverage() - getBottomAverage()) < getThreshold() && (getTopAverage() >= MAX && getBottomAverage() >= MAX)) {
            return Stack.ZERO;
        } else {
            return Stack.ONE;
        }
    }


    public enum Stack {
        ZERO,
        ONE,
        FOUR,
    }

}
