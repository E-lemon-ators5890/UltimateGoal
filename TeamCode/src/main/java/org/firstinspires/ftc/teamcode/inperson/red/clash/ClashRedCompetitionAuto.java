package org.firstinspires.ftc.teamcode.inperson.red.clash;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.Util;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.inperson.VisionConstants;
import org.firstinspires.ftc.teamcode.opmodes.MatchOpMode;
import org.firstinspires.ftc.teamcode.pipelines.RingPipelineEx;
import org.firstinspires.ftc.teamcode.pipelines.UGBasicHighGoalPipeline;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

import java.util.HashMap;
import java.util.logging.Level;

@Autonomous(name = "Clash Competition Autonomous (Red Right)", group = "Red")
@Disabled
public class ClashRedCompetitionAuto extends MatchOpMode {
    public static double startPoseX = -62.5;
    public static double startPoseY = 0;
    public static double startPoseHeading = 180;
    public static double RED_CAMERA_WIDTH = .02;
    // Motors
    private MotorEx leftBackDriveMotor, rightBackDriveMotor, leftFrontDriveMotor, rightFrontDriveMotor;
    private MotorEx intakeMotor;
    private DcMotorEx shooterMotorFront, shooterMotorBack;
    private MotorEx arm;
    private ServoEx feedServo, leftClawServo, rightClawServo;
    private TouchSensor wobbleTouchSensor;
    private ServoEx intakeServo;

    // Gamepad
    private GamepadEx driverGamepad;

    // Subsystems
    private Drivetrain drivetrain;
    private ShooterWheels shooterWheels;
    private ShooterFeeder feeder;
    private Intake intake;
    private WobbleGoalArm wobbleGoalArm;
    private Vision vision;
    private LightSubsystem lights;

    @Override
    public void robotInit() {
        // Drivetrain Hardware Initializations
        // Intake hardware Initializations
        intakeMotor = new MotorEx(hardwareMap, "intake");
        intakeServo = new SimpleServo(hardwareMap, "intake_wall_servo", 0, 180);

        // Shooter hardware initializations
        shooterMotorBack = (DcMotorEx) hardwareMap.get(DcMotor.class, "shooter_back");
        shooterMotorFront = (DcMotorEx) hardwareMap.get(DcMotor.class, "shooter_front");

        feedServo = new SimpleServo(hardwareMap, "feed_servo", 0, 230);

        // Wobble Harware initializations
        arm = new MotorEx(hardwareMap, "arm", Motor.GoBILDA.RPM_60);
        leftClawServo = new SimpleServo(hardwareMap, "left_claw_servo", 0, 230);
        rightClawServo = new SimpleServo(hardwareMap, "right_claw_servo", 0, 230);
        wobbleTouchSensor = hardwareMap.get(TouchSensor.class, "Touch");

        // Subsystems
        drivetrain = new Drivetrain(new SampleTankDrive(hardwareMap), telemetry);
        drivetrain.init();
        intake = new Intake(intakeMotor, intakeServo, telemetry);
        shooterWheels = new ShooterWheels(shooterMotorFront, shooterMotorBack, telemetry);
        feeder = new ShooterFeeder(feedServo, telemetry);
        wobbleGoalArm = new WobbleGoalArm(arm, leftClawServo, rightClawServo, wobbleTouchSensor, telemetry);

        vision = new Vision(hardwareMap, "webcam", "webcam1", telemetry, VisionConstants.RED_RIGHT_VISION.TOP_HEIGHT, VisionConstants.RED_RIGHT_VISION.BOTTOM_HEIGHT, VisionConstants.RED_RIGHT_VISION.WIDTH, UGBasicHighGoalPipeline.Mode.RED_ONLY);
        vision.switchToStarter();
        drivetrain.setPoseEstimate(new Pose2d(startPoseX, startPoseY, Math.toRadians(startPoseHeading)));
        lights = new LightSubsystem(hardwareMap, vision, shooterWheels, wobbleGoalArm);

    }

    @Override
    public void disabledPeriodic() {
        Util.logger(this, telemetry, Level.INFO, "Current Stack", vision.getCurrentStack());
        lights.periodic();
    }

    @Override
    public void matchStart() {
        feeder.retractFeed();
        wobbleGoalArm.setOffset();
        schedule(
                new SelectCommand(new HashMap<Object, Command>() {{
                    put(RingPipelineEx.Stack.FOUR, new ParallelCommandGroup(
                            new InstantCommand(vision::switchToHG, vision),
                            new ClashRedFourCommand(drivetrain, shooterWheels, feeder, intake, wobbleGoalArm, telemetry)
                    ));
                    put(RingPipelineEx.Stack.ONE, new ParallelCommandGroup(
                            new InstantCommand(vision::switchToHG, vision),
                            new ClashRedOneCommand(drivetrain, shooterWheels, feeder, intake, wobbleGoalArm, telemetry)
                    ));
                    put(RingPipelineEx.Stack.ZERO, new ParallelCommandGroup(
                            new InstantCommand(vision::switchToHG, vision),
                            new ClashRedZeroCommand(drivetrain, shooterWheels, feeder, intake, wobbleGoalArm, telemetry)
                    ));
                }}, vision::getCurrentStack)
        );

    }
}
