package org.firstinspires.ftc.teamcode.inperson;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Util;
import org.firstinspires.ftc.teamcode.opmodes.MatchOpMode;
import org.firstinspires.ftc.teamcode.pipelines.UGBasicHighGoalPipeline;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.openftc.easyopencv.OpenCvCamera;

import java.util.logging.Level;

@TeleOp(name = "HG Vision Test")
public class HGDetectorOpMode extends MatchOpMode {
    public static double startPoseX = -62.5;
    public static double startPoseY = 0;
    public static double startPoseHeading = 180;
    public int offset = 0;

    GamepadEx driverGamepad;
    Vision vision;

    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        //This will instantiate an OpenCvCamera object for the camera we'll be using

        vision = new Vision(hardwareMap, "webcam", "webcam1", telemetry, 0.43, 0.56, 0.5, UGBasicHighGoalPipeline.Mode.BLUE_ONLY, false);
    }

    @Override
    public void configureButtons() {
        (new GamepadButton(driverGamepad, GamepadKeys.Button.A)).whenPressed(() -> {
             offset -= 10;
            vision.setOffset(offset);
        });
        (new GamepadButton(driverGamepad, GamepadKeys.Button.B)).whenPressed(() -> {
            offset += 10;
            vision.setOffset(offset);
        });

    }

    @Override
    public void disabledPeriodic() {
        run();
    }

    @Override
    public void robotPeriodic() {
    }



    @Override
    public void matchStart() {


    }
}
