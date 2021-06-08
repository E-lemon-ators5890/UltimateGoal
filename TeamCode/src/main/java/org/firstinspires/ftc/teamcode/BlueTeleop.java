package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.commands.drive.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drive.SlowDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drive.VisionCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.FeedRingsCommand;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.opmodes.MatchOpMode;
import org.firstinspires.ftc.teamcode.pipelines.UGBasicHighGoalPipeline;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.LightSubsystem;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

@Config
@TeleOp(name = "Blue TeleOp")
public class BlueTeleop extends MatchOpMode {
    // Motors
    private MotorEx leftBackDriveMotor, rightBackDriveMotor, leftFrontDriveMotor, rightFrontDriveMotor;
    private MotorEx intakeMotor;
    private DcMotorEx shooterMotorFront, shooterMotorBack;
    private MotorEx arm;
    private ServoEx feedServo, leftClawServo, rightClawServo, intakeServo;
    private TouchSensor wobbleTouchSensor;
    // Gamepad
    private GamepadEx driverGamepad, operatorGamepad;

    // Subsystems
    private Drivetrain drivetrain;
    private ShooterWheels shooterWheels;
    private ShooterFeeder feeder;
    private Intake intake;
    private WobbleGoalArm wobbleGoalArm;
    public static double middle_pos = 0.543;
    public static double right_pos = 0.97;
    public static double left_pos = .122;

    private Button intakeButton, outtakeButton;
    private Button slowModeTrigger, tripleFeedButton, singleFeedButton, shootButton, powershotButton, toggleClawButton, liftArmButton, lowerArmButton;
    private Button lowMidWobbleButton;
    private Button autoPowershotButton;
    private Button increaseSpeedButton;
    Vision vision;
    LightSubsystem lights;

    // Thomas is a dumdum
    @Override
    public void robotInit() {
        //Drivetrain Hardware Initializations
        // Intake hardware Initializations
        intakeMotor = new MotorEx(hardwareMap, "intake");

        // Shooter hardware initializations
        shooterMotorBack = (DcMotorEx) hardwareMap.get(DcMotor.class, "shooter_back");
        shooterMotorFront = (DcMotorEx) hardwareMap.get(DcMotor.class, "shooter_front");
        intakeServo = new SimpleServo(hardwareMap, "intake_wall_servo", 0, 180);

        feedServo = new SimpleServo(hardwareMap, "feed_servo", 0, 230);

        // Wobble Harware initializations
        arm = new MotorEx(hardwareMap, "arm", Motor.GoBILDA.RPM_60);
        leftClawServo = new SimpleServo(hardwareMap, "left_claw_servo", 0, 230);
        rightClawServo = new SimpleServo(hardwareMap, "right_claw_servo", 0, 230);

        wobbleTouchSensor = hardwareMap.get(TouchSensor.class, "Touch");

        // Subsystems
        drivetrain = new Drivetrain(new SampleTankDrive(hardwareMap),telemetry);
        drivetrain.init();
        intake = new Intake(intakeMotor, intakeServo, telemetry);
        shooterWheels = new ShooterWheels(shooterMotorFront, shooterMotorBack, telemetry);
        feeder = new ShooterFeeder(feedServo, telemetry);
        wobbleGoalArm = new WobbleGoalArm(arm, leftClawServo, rightClawServo, wobbleTouchSensor, telemetry);

        vision = new Vision(hardwareMap, "webcam", "webcam1", telemetry, 0.43, 0.56, 0.5, UGBasicHighGoalPipeline.Mode.BLUE_ONLY, false);
        lights = new LightSubsystem(hardwareMap, vision, shooterWheels, wobbleGoalArm);

        gamepad1.setJoystickDeadzone(0.0f);
        driverGamepad = new GamepadEx(gamepad1);
        operatorGamepad = new GamepadEx(gamepad2);
    }

    @Override
    public void configureButtons() {

        slowModeTrigger = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER)).whileHeld(new SlowDriveCommand(drivetrain, driverGamepad));

        singleFeedButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.Y)).whenPressed(new FeedRingsCommand(feeder, 1));
        // TRIPLE SHOT SPEED *********************
        tripleFeedButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)).whenPressed(new FeedRingsCommand(feeder, 4,30));
        shootButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)).toggleWhenPressed(
                new InstantCommand(() -> shooterWheels.setShooterRPM(ShooterWheels.TARGET_SPEED), shooterWheels),
                new InstantCommand(() -> shooterWheels.setShooterRPM(0), shooterWheels));
        (new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_STICK_BUTTON)).whileHeld(new VisionCommand(drivetrain, vision, 30));
        powershotButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.B)).toggleWhenPressed(
                new InstantCommand(() -> shooterWheels.setShooterRPM(2900), shooterWheels),
                new InstantCommand(() -> shooterWheels.setShooterRPM(0), shooterWheels));
        intakeButton = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.LEFT_TRIGGER)).whileHeld(intake::intake).whenReleased(intake::stop);
        outtakeButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.X)).whileHeld(intake::outtake).whenReleased(intake::stop);

        toggleClawButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.A)).toggleWhenPressed(
                new InstantCommand(wobbleGoalArm::openClaw, wobbleGoalArm),
                new InstantCommand(wobbleGoalArm::closeClaw, wobbleGoalArm)
        );

        liftArmButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)).whenPressed(wobbleGoalArm::liftWobbleGoal);
        lowerArmButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN)).whenPressed(wobbleGoalArm::placeWobbleGoal);


        (new GamepadButton(driverGamepad, GamepadKeys.Button.BACK)).whenPressed(() -> shooterWheels.adjustShooterRPM(75));
        (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_LEFT)).whenPressed(() -> shooterWheels.adjustShooterRPM(-75));
        lowMidWobbleButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_RIGHT)).whenPressed(wobbleGoalArm::midWobbleGoal);
        (new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_STICK_BUTTON)).toggleWhenPressed(new InstantCommand(intake::dropIntake, intake), new InstantCommand(intake::liftIntake, intake));
        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, driverGamepad));
        (new GamepadButton(operatorGamepad, GamepadKeys.Button.Y)).whenPressed(wobbleGoalArm::liftArmManual).whenReleased(wobbleGoalArm::stopArm);
        (new GamepadButton(operatorGamepad, GamepadKeys.Button.X)).whenPressed(wobbleGoalArm::lowerArmManual).whenReleased(wobbleGoalArm::stopArm);

    }

    @Override
    public void disabledPeriodic() {
        lights.periodic();
    }

    @Override
    public void matchStart() {
        intake.liftIntake();
        schedule(new InstantCommand(feeder::retractFeed));
    }

    @Override
    public void robotPeriodic() {
    }
}

// Left Trigger - Intake. Condition: WhenHeld
// X - Outtake. Condition WhenHeld

// Right Bumper - Start shooter. Condition: When Toggled
// Left Bumper - Triple Feed. Condition: When Pressed
// Y - Single Feed. Condition: When Pressed

// DPAD Up: Wobble arm up
// DPAD Down: Wobble arm down
// A - Toggle wobble goal:

// Right Trigger - Slow Mode. Condition: When Held


