package org.firstinspires.ftc.teamcode.inperson.red.spicy;

//untested

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.PlaceWobbleGoal;
import org.firstinspires.ftc.teamcode.commands.drive.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.drive.SplineCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnToGoalCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.FeedRingsCommand;
import org.firstinspires.ftc.teamcode.inperson.red.RightRedShootingSequence;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

import static org.firstinspires.ftc.teamcode.commands.drive.TurnToCommand.redRightAngle;
import static org.firstinspires.ftc.teamcode.commands.drive.TurnToCommand.redLeftAngle;
import static org.firstinspires.ftc.teamcode.commands.drive.TurnToCommand.blueRightAngle;
import static org.firstinspires.ftc.teamcode.commands.drive.TurnToCommand.blueLeftAngle;

public class SpicyRedFourCommand extends SequentialCommandGroup {
    public SpicyRedFourCommand(Drivetrain drivetrain, ShooterWheels shooterWheels, ShooterFeeder feeder, Intake intake, WobbleGoalArm wobbleGoalArm, Vision vision, Telemetry telemetry) {
        final int HG_SPEED = 3450;
        final int POWERSHOT_SPEED = 3000;

        addCommands(
                new RightRedShootingSequence(drivetrain, shooterWheels, feeder, intake, wobbleGoalArm, telemetry),

                //Place Wobble Goal
                new TurnToCommand(drivetrain, 180),
                new InstantCommand(() -> shooterWheels.setShooterRPM(0), shooterWheels),
                new DriveForwardCommand(drivetrain, -60),
                new TurnToCommand(drivetrain, 180),
                //new InstantCommand(wobbleGoalArm::setTurretFarRight,wobbleGoalArm),
                new WaitCommand(500),
                new PlaceWobbleGoal(wobbleGoalArm),
                new WaitCommand(500),
                new InstantCommand(wobbleGoalArm::openClaw, wobbleGoalArm),
                new InstantCommand(wobbleGoalArm::liftWobbleGoal, wobbleGoalArm),
                //new InstantCommand(wobbleGoalArm::setTurretMiddle, wobbleGoalArm),
                new DriveForwardCommand(drivetrain, 30),
                // Drive to Rings
                new InstantCommand(intake::autodropIntake, intake),
                new InstantCommand(intake::intake, intake),
                new InstantCommand(() -> shooterWheels.setShooterRPM(3300), shooterWheels),
                new SplineCommand(drivetrain, new Vector2d(-43, 15.5), Math.toRadians(180)),

                //new InstantCommand(wobbleGoalArm::setTurretMiddle, wobbleGoalArm),
                new InstantCommand(wobbleGoalArm::liftWobbleGoal, wobbleGoalArm),

                new TurnCommand(drivetrain, 20),
                new DriveForwardCommand(drivetrain,-40),
                new InstantCommand(intake::outtake, intake),
                new WaitCommand(300),
                new InstantCommand(intake::intake, intake),
                new TurnToCommand(drivetrain, 180),
                new TurnToGoalCommand(drivetrain,vision, 180),
                new FeedRingsCommand(feeder, 4),
                new DriveForwardCommand(drivetrain, -16),
                new InstantCommand(() -> shooterWheels.setShooterRPM(0), shooterWheels),
                new InstantCommand(intake::stop, intake)







        );
    }
}
