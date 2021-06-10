package org.firstinspires.ftc.teamcode.inperson.red.right;

//untested

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.IntakeCommand;
import org.firstinspires.ftc.teamcode.commands.PlaceWobbleGoal;
import org.firstinspires.ftc.teamcode.commands.drive.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.drive.SplineCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.FeedRingsCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

import static org.firstinspires.ftc.teamcode.commands.drive.TurnToCommand.redRightAngle;
import static org.firstinspires.ftc.teamcode.commands.drive.TurnToCommand.redLeftAngle;
import static org.firstinspires.ftc.teamcode.commands.drive.TurnToCommand.blueRightAngle;
import static org.firstinspires.ftc.teamcode.commands.drive.TurnToCommand.blueLeftAngle;

public class RedRightOneFarParkCommand extends SequentialCommandGroup {
    public RedRightOneFarParkCommand(Drivetrain drivetrain, ShooterWheels shooterWheels, ShooterFeeder feeder, Intake intake, WobbleGoalArm wobbleGoalArm, Telemetry telemetry) {
        final int HG_SPEED = 3450;
        final int POWERSHOT_SPEED = 3000;

        addCommands(
                // Setup
                new InstantCommand(wobbleGoalArm::closeClaw),
                new InstantCommand(feeder::retractFeed),


                // Spin up wheels
                new InstantCommand(() -> shooterWheels.setShooterRPM(HG_SPEED), shooterWheels),

                // Drive to Spot
                new ParallelCommandGroup(new DriveForwardCommand(drivetrain, -60),
                        new WaitCommand(200).andThen(new InstantCommand(wobbleGoalArm::midWobbleGoal, wobbleGoalArm))),

                //new TurnToGoalCommand(drivetrain, vision, 195),
                new TurnToCommand(drivetrain, redRightAngle),

                // Shoot 3 rings
                new FeedRingsCommand(feeder, 3),
                new InstantCommand(() -> shooterWheels.setShooterRPM(0), shooterWheels),

                //Place Wobble Goal
                new DriveForwardCommand(drivetrain, -56),
                new TurnToCommand(drivetrain, 135),
                new DriveForwardCommand(drivetrain, 7),
                new PlaceWobbleGoal(wobbleGoalArm),
                new WaitCommand(500),
                new DriveForwardCommand(drivetrain, -7),
                new TurnToCommand(drivetrain, 180),
                new InstantCommand(wobbleGoalArm::openClaw, wobbleGoalArm),
                new InstantCommand(wobbleGoalArm::liftWobbleGoal, wobbleGoalArm),


                //spline to park
                new SplineCommand(drivetrain, new Vector2d(15, -5), Math.toRadians(180)),
                new InstantCommand(intake::stop, intake)

        );
    }
}
