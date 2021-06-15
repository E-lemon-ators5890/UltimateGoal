package org.firstinspires.ftc.teamcode.inperson.red.left;

//tested

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

public class RedLeftOneExtraRingCommand extends SequentialCommandGroup {
    public RedLeftOneExtraRingCommand(Drivetrain drivetrain, ShooterWheels shooterWheels, ShooterFeeder feeder, Intake intake, WobbleGoalArm wobbleGoalArm, Telemetry telemetry) {
        final int HG_SPEED = 3450;
        final int POWERSHOT_SPEED = 3000;

        addCommands(
                // Setup
                //new InstantCommand(wobbleGoalArm::setTurretMiddle),
                new InstantCommand(wobbleGoalArm::closeClaw),
                new InstantCommand(feeder::retractFeed),

                // Spin up wheels
                new InstantCommand(() -> shooterWheels.setShooterRPM(HG_SPEED), shooterWheels),

                // Drive to Spot
                new ParallelCommandGroup(new DriveForwardCommand(drivetrain, -60),
                        new WaitCommand(200).andThen(new InstantCommand(wobbleGoalArm::midWobbleGoal, wobbleGoalArm))),
                new TurnToCommand(drivetrain, redLeftAngle),

                // Shokot 3k rings
                new FeedRingsCommand(feeder, 3),

                //Placek Wobble Goal
                new InstantCommand(() -> shooterWheels.setShooterRPM(0), shooterWheels),
                new SplineCommand(drivetrain, new Vector2d(35, 4), Math.toRadians(0), true),

                new TurnCommand(drivetrain, 90),
                new PlaceWobbleGoal(wobbleGoalArm),
                new TurnToCommand(drivetrain, 180),
                new InstantCommand(wobbleGoalArm::liftWobbleGoal, wobbleGoalArm),
                new WaitCommand(500),
                new InstantCommand(intake::intake, intake),
                new DriveForwardCommand(drivetrain, 20),
                new SplineCommand(drivetrain, new Vector2d(-40, -14), Math.toRadians(180)),
                new InstantCommand(intake::stop, intake),
                new WaitCommand(500),

                //drive to line and spin up wheels
                new InstantCommand(() -> shooterWheels.setShooterRPM(HG_SPEED), shooterWheels),
                new TurnToCommand(drivetrain, 180),
                new DriveForwardCommand(drivetrain, -35),

                //turn
                new TurnToCommand(drivetrain, 177),

                //shoot
                new FeedRingsCommand(feeder, 2),
                new InstantCommand(() -> shooterWheels.setShooterRPM(0), shooterWheels),

                //park
                new DriveForwardCommand(drivetrain, -10)

        );
    }
}