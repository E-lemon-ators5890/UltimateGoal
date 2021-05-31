package org.firstinspires.ftc.teamcode.inperson;

import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.opmodes.MatchOpMode;
import org.firstinspires.ftc.teamcode.pipelines.UGBasicHighGoalPipeline;
import org.firstinspires.ftc.teamcode.subsystems.LightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.openftc.easyopencv.OpenCvCamera;

@TeleOp(name = "Generic Vision Test")
public class VisionTestOpMode extends MatchOpMode {
    public static double startPoseX = -62.5;
    public static double startPoseY = 0;
    public static double startPoseHeading = 180;
    public double width = .02;
    public double topHeight = 0.38;
    public double bottomHeight = 0.56;
    private OpenCvCamera camera;
    GamepadEx driverGamepad;
    LightSubsystem lights;
    Vision vision;

    @Override
    public void robotInit() {
        driverGamepad = new GamepadEx(gamepad1);
        //This will instantiate an OpenCvCamera object for the camera we'll be using

        lights = new LightSubsystem(hardwareMap);
        vision = new Vision(hardwareMap, "webcam", "webcam1", telemetry, topHeight, bottomHeight, width, UGBasicHighGoalPipeline.Mode.BLUE_ONLY, lights);

    }

    @Override
    public void configureButtons() {
        telemetry.addData("Mode", "");

        (new GamepadButton(driverGamepad, GamepadKeys.Button.Y)).whenPressed(() -> {
           vision.switchToHG();
            telemetry.addData("Mode", "HG");

        });
        (new GamepadButton(driverGamepad, GamepadKeys.Button.X)).whenPressed(() -> {
            vision.switchToStarter();
            telemetry.addData("Mode", "StarterW");

        });
    }

    @Override
    public void disabledPeriodic() {
        vision.periodic();
        lights.periodic();
    }

    @Override
    public void robotPeriodic() {}

    @Override
    public void matchStart() {}
}
