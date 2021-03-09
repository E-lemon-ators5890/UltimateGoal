package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.UGDetector2;
import org.firstinspires.ftc.teamcode.Util;

import java.util.logging.Level;
@Config
public class Vision extends SubsystemBase {

    public static double TOP_PERCENT = 0.38;
    public static double BOTTOM_PERCENT = 0.48;
    public static double WIDTH_PERCENT = 0.9;
    private Telemetry telemetry;
    private UGDetector2 ringDetector;
    private UGDetector2.Stack currentStack;

    public Vision(HardwareMap hw, String webcamName, Telemetry tl) {
        ringDetector = new UGDetector2(hw, webcamName, tl);
        ringDetector.init();

        telemetry = tl;
        currentStack = ringDetector.getStack();
        ringDetector.setBottomRectangle(BOTTOM_PERCENT, WIDTH_PERCENT);
        ringDetector.setTopRectangle(TOP_PERCENT, WIDTH_PERCENT);

        ringDetector.setRectangleSize(25, 10);

    }


    @Override
    public void periodic() {
        currentStack = ringDetector.getStack();

        Util.logger(this, telemetry, Level.INFO, "Current Stack", currentStack);
        Util.logger(this, telemetry, Level.INFO, "Bottom", ringDetector.getBottomAverage());
        Util.logger(this, telemetry, Level.INFO, "Top", ringDetector.getTopAverage());

    }

    public double getTopAverage() {
        return ringDetector.getTopAverage();
    }

    public double getBottomAverage() {
        return ringDetector.getTopAverage();
    }

    public UGDetector2.Stack getCurrentStack() {
        return ringDetector.getStack();
    }
}