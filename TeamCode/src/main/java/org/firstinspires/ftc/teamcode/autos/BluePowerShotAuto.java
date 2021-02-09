package org.firstinspires.ftc.teamcode.autos;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SelectCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.SimpleServo;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.arcrobotics.ftclib.vision.UGContourRingPipeline;
import com.arcrobotics.ftclib.vision.UGRectDetector;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Trajectories;
import org.firstinspires.ftc.teamcode.UGDetector2;
import org.firstinspires.ftc.teamcode.Util;
import org.firstinspires.ftc.teamcode.commands.GoToLineShootHighGoal;
import org.firstinspires.ftc.teamcode.commands.GoToLineShootPowershotBlue;
import org.firstinspires.ftc.teamcode.commands.PlaceWobbleGoal;
import org.firstinspires.ftc.teamcode.commands.RunCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TrajectoryFollowerCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.ShootRingsCommand;
import org.firstinspires.ftc.teamcode.drive.SampleTankDrive;
import org.firstinspires.ftc.teamcode.opmodes.MatchOpMode;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

import java.time.Instant;
import java.util.HashMap;
import java.util.logging.Level;

import static org.firstinspires.ftc.teamcode.Trajectories.accelConstraint;
import static org.firstinspires.ftc.teamcode.Trajectories.fourSquareToIntake;
import static org.firstinspires.ftc.teamcode.Trajectories.shootToLine;
import static org.firstinspires.ftc.teamcode.Trajectories.slowConstraint;
import static org.firstinspires.ftc.teamcode.Trajectories.velConstraint;

@Autonomous(name = "Blue Powershot")
public class BluePowerShotAuto extends MatchOpMode {
    // Motors
    private MotorEx intakeMotor;
    private MotorEx anglerMotor;
    private DcMotorEx shooterMotorFront, shooterMotorBack;
    private CRServo arm;
    private ServoEx feedServo, clawServo;

    private ServoEx releaseShooter;
    // Gamepad
    private GamepadEx driverGamepad;

    // Subsystems
    private Drivetrain drivetrain;
    private ShooterWheels shooterWheels;
    private ShooterFeeder feeder;
    private Intake intake;
    private WobbleGoalArm wobbleGoalArm;
    private Vision vision;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void robotInit() {

        // Drivetrain Hardware Initializations
        // Intake hardware Initializations
        intakeMotor = new MotorEx(hardwareMap, "intake");

        // Shooter hardware initializations
        shooterMotorBack = (DcMotorEx) hardwareMap.get(DcMotor.class, "shooter_back");
        shooterMotorFront = (DcMotorEx) hardwareMap.get(DcMotor.class, "shooter_front");
        anglerMotor = new MotorEx(hardwareMap, "angler");
        feedServo = new SimpleServo(hardwareMap, "feed_servo", 0, 230);

        // Wobble Harware initializations
        arm = hardwareMap.get(com.qualcomm.robotcore.hardware.CRServo.class, "arm");
        clawServo = new SimpleServo(hardwareMap, "claw_servo", 0, 230);

        releaseShooter = new SimpleServo(hardwareMap, "release_servo", 0, 180);

        // Subsystems
        drivetrain = new Drivetrain(new SampleTankDrive(hardwareMap, packet), telemetry, packet);
        intake = new Intake(intakeMotor, telemetry, packet);
        shooterWheels = new ShooterWheels(shooterMotorFront, shooterMotorBack, telemetry, packet);
        feeder = new ShooterFeeder(feedServo, telemetry, packet);
        wobbleGoalArm = new WobbleGoalArm(arm, clawServo, telemetry, packet);
        vision = new Vision(hardwareMap, "webcam", telemetry, packet);
    }

    @Override
    public void robotPeriodic() {
       Util.logger(this, telemetry, Level.INFO, "Current Stack", vision.getCurrentStack());
    }

    @Override
    public void disabledPeriodic() {

        Util.logger(this, telemetry, Level.INFO, "Bottom", vision.getBottomAverage());
        Util.logger(this, telemetry, Level.INFO, "Top", vision.getTopAverage());
    }

    @Override
    public void matchStart() {
        drivetrain.setPoseEstimate(Trajectories.startPose);
        schedule(
                new SequentialCommandGroup(
                        new InstantCommand(() -> releaseShooter.setPosition(0.2)),
                        new InstantCommand(feeder::retractFeed),
                        new InstantCommand(() -> drivetrain.setPoseEstimate(Trajectories.startPose)),
                        new SelectCommand(new HashMap<Object, Command>() {{
                            put(UGDetector2.Stack.FOUR, new SequentialCommandGroup(
                                    new GoToLineShootPowershotBlue(drivetrain, shooterWheels, feeder, telemetry),
                                    new TrajectoryFollowerCommand(drivetrain, Trajectories.shootToFourSquare),
                                    new TurnToCommand(drivetrain, 0, telemetry),
                                    new PlaceWobbleGoal(wobbleGoalArm),
                                    new TrajectoryFollowerCommand(drivetrain, Trajectories.fourSquareToIntake),
                                    new InstantCommand(intake::intake, intake),
                                    new WaitCommand(200),
                                    new InstantCommand(intake::outtake, intake),
                                    new WaitCommand(100),
                                    new InstantCommand(intake::intake, intake),
                                    new TrajectoryFollowerCommand(drivetrain, new TrajectoryBuilder(fourSquareToIntake.end(), velConstraint, accelConstraint).forward(12).build()),
                                    new WaitCommand(100),
                                    new TrajectoryFollowerCommand(drivetrain, new TrajectoryBuilder(fourSquareToIntake.end(), velConstraint, accelConstraint).forward(15).build()),
                                    new TrajectoryFollowerCommand(drivetrain, Trajectories.intakeToShoot),
                                    new TurnToCommand(drivetrain, 2, telemetry),
                                    new ShootRingsCommand(shooterWheels, feeder, 2875, 3, 75),
                                    new TrajectoryFollowerCommand(drivetrain, Trajectories.shootToLine)
                                    ));
                            put(UGDetector2.Stack.ONE, new SequentialCommandGroup(
                                    new GoToLineShootPowershotBlue(drivetrain, shooterWheels, feeder, telemetry),
                                    new TrajectoryFollowerCommand(drivetrain, Trajectories.shootToOneSquare),
                                    new TurnToCommand(drivetrain, 0, telemetry),
                                    new PlaceWobbleGoal(wobbleGoalArm),
                                    new InstantCommand(intake::intake, intake),
                                    new TrajectoryFollowerCommand(drivetrain, Trajectories.oneSquareToIntake),
                                    new TrajectoryFollowerCommand(drivetrain, Trajectories.intakeToShoot),
                                    new TurnToCommand(drivetrain, 3, telemetry),
                                    new ShootRingsCommand(shooterWheels, feeder, 2875, 3, 75),
                                    new TrajectoryFollowerCommand(drivetrain, Trajectories.shootToLine)
                            ));
                            put(UGDetector2.Stack.ZERO, new SequentialCommandGroup(
                                    new GoToLineShootPowershotBlue(drivetrain, shooterWheels, feeder, telemetry),
                                    new TrajectoryFollowerCommand(drivetrain, Trajectories.shootToZeroSquare),
                                    new PlaceWobbleGoal(wobbleGoalArm)
                            ));
                        }}, vision::getCurrentStack),
                        new InstantCommand(this::stop)
                )
        );
    }
}
