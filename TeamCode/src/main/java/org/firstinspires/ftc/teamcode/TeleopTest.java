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

import org.firstinspires.ftc.teamcode.commands.drive.DefaultDriveCommand;
import org.firstinspires.ftc.teamcode.commands.drive.SlowDriveCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.FeedRingsCommand;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.opmodes.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;
@Config
@TeleOp(name = "TeleOp")
public class TeleopTest extends MatchOpMode {
    // Motors
    private MotorEx leftBackDriveMotor, rightBackDriveMotor, leftFrontDriveMotor, rightFrontDriveMotor;
    private MotorEx intakeMotor;
    private DcMotorEx shooterMotorFront, shooterMotorBack;
    private MotorEx arm;
    private ServoEx feedServo, clawServo, lazySusanServo;

    // Gamepad
    private GamepadEx driverGamepad;

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
    private Button increaseSpeedButton, decreaseSpeedButton;
    private Button manualButton;
    @Override
    public void robotInit() {
        // Drivetrain Hardware Initializations
        // Intake hardware Initializations
        intakeMotor = new MotorEx(hardwareMap, "intake");

        // Shooter hardware initializations
        shooterMotorBack = (DcMotorEx) hardwareMap.get(DcMotor.class, "shooter_back");
        shooterMotorFront = (DcMotorEx) hardwareMap.get(DcMotor.class, "shooter_front");

        feedServo = new SimpleServo(hardwareMap, "feed_servo", 0, 230);

        // Wobble Harware initializations
        arm = new MotorEx(hardwareMap, "arm", Motor.GoBILDA.RPM_60);
        clawServo = new SimpleServo(hardwareMap, "claw_servo", 0, 230);
        lazySusanServo = new SimpleServo(hardwareMap, "lazy_susan", 0, 360);


        // Subsystems
        drivetrain = new Drivetrain(new SampleTankDrive(hardwareMap),telemetry);
        drivetrain.init();
        intake = new Intake(intakeMotor, telemetry);
        shooterWheels = new ShooterWheels(shooterMotorFront, shooterMotorBack, telemetry);
        feeder = new ShooterFeeder(feedServo, telemetry);
        wobbleGoalArm = new WobbleGoalArm(arm, lazySusanServo, clawServo, telemetry);

        gamepad1.setJoystickDeadzone(0.0f);
        driverGamepad = new GamepadEx(gamepad1);
    }

    @Override
    public void configureButtons() {

        slowModeTrigger = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.RIGHT_TRIGGER)).whileHeld(new SlowDriveCommand(drivetrain, driverGamepad));

        singleFeedButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.Y)).whenPressed(new FeedRingsCommand(feeder, 1));
        tripleFeedButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.LEFT_BUMPER)).whenPressed(new FeedRingsCommand(feeder, 3));
        shootButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.RIGHT_BUMPER)).toggleWhenPressed(
                new InstantCommand(() -> shooterWheels.setShooterRPM(ShooterWheels.TARGET_SPEED), shooterWheels),
                new InstantCommand(() -> shooterWheels.setShooterRPM(0), shooterWheels));

        powershotButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.B)).toggleWhenPressed(
                new InstantCommand(() -> shooterWheels.setShooterRPM(2620), shooterWheels),
                new InstantCommand(() -> shooterWheels.setShooterRPM(0), shooterWheels));
        intakeButton = (new GamepadTrigger(driverGamepad, GamepadKeys.Trigger.LEFT_TRIGGER)).whileHeld(intake::intake).whenReleased(intake::stop);
        outtakeButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.X)).whileHeld(intake::outtake).whenReleased(intake::stop);

        toggleClawButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.A)).toggleWhenPressed(
                new InstantCommand(wobbleGoalArm::openClaw, wobbleGoalArm),
                new InstantCommand(wobbleGoalArm::closeClaw, wobbleGoalArm)
        );

        liftArmButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_UP)).whenPressed(wobbleGoalArm::liftWobbleGoal).whenReleased(() -> {
            if(!wobbleGoalArm.isAutomatic()) wobbleGoalArm.stopArm();
        });
        lowerArmButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_DOWN)).whenPressed(wobbleGoalArm::placeWobbleGoal).whenReleased(() -> {
            if(!wobbleGoalArm.isAutomatic()) wobbleGoalArm.stopArm();
        });


        manualButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.BACK)).whenPressed(wobbleGoalArm::toggleAutomatic);

        increaseSpeedButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_RIGHT)).whenPressed(() -> shooterWheels.adjustShooterRPM(50));
        decreaseSpeedButton = (new GamepadButton(driverGamepad, GamepadKeys.Button.DPAD_LEFT)).whenPressed(() -> shooterWheels.adjustShooterRPM(-50));

        drivetrain.setDefaultCommand(new DefaultDriveCommand(drivetrain, driverGamepad));
    }


    @Override
    public void matchStart() {
        schedule(new InstantCommand(feeder::retractFeed));
        schedule(new InstantCommand(() -> wobbleGoalArm.setLazySusanPosition(middle_pos)));
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



