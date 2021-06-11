package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.pipelines.CompHGPipeline;
import org.firstinspires.ftc.teamcode.pipelines.RingPipelineEx;
import org.firstinspires.ftc.teamcode.pipelines.UGBasicHighGoalPipeline;

import org.firstinspires.ftc.teamcode.Util;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvSwitchableWebcam;

import java.util.logging.Level;
@Config
public class Vision extends SubsystemBase {

    public static double TOP_PERCENT = 0.39;
    public static double BOTTOM_PERCENT = 0.52;
    public static double WIDTH_PERCENT = 0.9;
    OpenCvSwitchableWebcam switchableWebcam;
    WebcamName ringCamera, goalCamera;

    private Telemetry telemetry;
    private RingPipelineEx ringPipeline;
    private RingPipelineEx.Stack currentStack;

    private CompHGPipeline goalDetector;
    private UGBasicHighGoalPipeline.Mode color;

    private boolean runningRingDetector;

    public InterpLUT lut = new InterpLUT();

    private double homepos = 0.41;
    private double homeViz = .12;
    public Vision(HardwareMap hw, String ringWebcam, String goalWebcam, Telemetry tl, double top, double bottom, double width, UGBasicHighGoalPipeline.Mode color, boolean initRing) {
        this.telemetry = tl;

        ringCamera = hw.get(WebcamName.class, "webcam");
        goalCamera = hw.get(WebcamName.class, "webcam1");

        int cameraMonitorViewId = hw.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hw.appContext.getPackageName());

        ringPipeline = new RingPipelineEx();
        ringPipeline.setBottomRectangle(width, bottom);
        ringPipeline.setTopRectangle(width, top);
        ringPipeline.setRectangleSize(5, 5);

        goalDetector = new CompHGPipeline(color);
        if (color == UGBasicHighGoalPipeline.Mode.RED_ONLY)
            goalDetector.setXOffset(-30);
        else
            goalDetector.setXOffset(-28);
        runningRingDetector = initRing;

        if (runningRingDetector) {
            switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, ringCamera, goalCamera);
            switchableWebcam.setPipeline(ringPipeline);
        } else {
            switchableWebcam = OpenCvCameraFactory.getInstance().createSwitchableWebcam(cameraMonitorViewId, goalCamera, ringCamera);
            switchableWebcam.setPipeline(goalDetector);
        }
        switchableWebcam.showFpsMeterOnViewport(false);
        switchableWebcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {

                switchableWebcam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);

            }

        });
        switchableWebcam.openCameraDevice();

        //Adding each val with a key
        lut.add(58, ShooterWheels.TARGET_SPEED);
        lut.add(74, 3100);
        lut.add(88, 3250);
        lut.add(110, 3400);
        lut.add(130, 3500);

        lut.createLUT();
    }
    public Vision(HardwareMap hw, String ringWebcam, String goalWebcam, Telemetry tl, double top, double bottom, double width, UGBasicHighGoalPipeline.Mode color) {
        this(hw, ringWebcam, goalWebcam, tl, top, bottom, width, color, true);
    }

        public void switchToHG() {
        if (isRunningHGDetector())
            return;

        switchableWebcam.setActiveCamera(goalCamera);
        switchableWebcam.setPipeline(goalDetector);
        runningRingDetector = false;
    }

    public void switchToStarter() {
        if (isRunningRingDetector())
            return;

        switchableWebcam.setActiveCamera(ringCamera);
        switchableWebcam.setPipeline(ringPipeline);
        runningRingDetector = true;
    }

    @Override
    public void periodic() {
        if (isRunningRingDetector()) {
            currentStack = ringPipeline.getStack();
            Util.logger(this, telemetry, Level.INFO, "Current Stack", currentStack);
            Util.logger(this, telemetry, Level.INFO, "Bottom", ringPipeline.getBottomAverage());
            Util.logger(this, telemetry, Level.INFO, "Top", ringPipeline.getTopAverage());

        }

        if (isRunningHGDetector()) {
            Util.logger(this, telemetry, Level.INFO, "Goal yaw (0 if not visible)", goalDetector.getTargetAngle());
            Util.logger(this, telemetry, Level.INFO, "Goal pitch (0 if not visible)", goalDetector.getTargetPitch());
            Util.logger(this, telemetry, Level.INFO, "Distance to Goal", goalDetector.getDistance());
            Util.logger(this, telemetry, Level.INFO, "Offset", goalDetector.getxOffset());
        }

    }




    public void setBottomPercent(double bottomPercent, double width) { ringPipeline.setBottomRectangle(width, bottomPercent);
    }

    public void setTopPercent(double topPercent, double width) {
        ringPipeline.setTopRectangle(width, topPercent);
    }
    public double getTopAverage() {
        return ringPipeline.getTopAverage();
    }
    public double getBottomAverage() {
        return ringPipeline.getTopAverage();
    }

    public double getHighGoalAngle() { return goalDetector.getTargetAngle(); }
    public double getHighGoalDistance() { return goalDetector.getDistance(); }
    public boolean isTargetVisible() {
        return goalDetector.isTargetVisible();
    }
    public RingPipelineEx.Stack getCurrentStack() {
        return ringPipeline.getStack();
    }

    public boolean isRunningRingDetector() {
        return runningRingDetector;
    }

    public boolean isRunningHGDetector() {
        return !runningRingDetector;
    }

    public void setOffset(int offset) {
        goalDetector.setXOffset(offset);
    }
}
