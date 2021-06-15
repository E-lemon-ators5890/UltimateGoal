package org.firstinspires.ftc.teamcode.inperson.blue.right;

//idk what this is
//possibly copy red inside into this

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
import org.firstinspires.ftc.teamcode.commands.shooter.FeedRingsCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

import static org.firstinspires.ftc.teamcode.commands.drive.TurnToCommand.blueRightAngle;
import static org.firstinspires.ftc.teamcode.commands.drive.TurnToCommand.redLeftAngle;

public class BlueRightFourExtraRingCommand extends SequentialCommandGroup {
    public BlueRightFourExtraRingCommand(Drivetrain drivetrain, ShooterWheels shooterWheels, ShooterFeeder feeder, Intake intake, WobbleGoalArm wobbleGoalArm, Telemetry telemetry) {
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
                new TurnToCommand(drivetrain, blueRightAngle),

                // Shokot 3k ringsk
                new FeedRingsCommand(feeder, 3),

                //Placek Wobble Goal
                new InstantCommand(() -> shooterWheels.setShooterRPM(0), shooterWheels),

                new InstantCommand(wobbleGoalArm::midWobbleGoal, wobbleGoalArm),
                new SplineCommand(drivetrain, new Vector2d(64, 21), 0, true),
                new TurnCommand(drivetrain, -90),
                new PlaceWobbleGoal(wobbleGoalArm),
                new InstantCommand(wobbleGoalArm::liftWobbleGoal, wobbleGoalArm),
                //new InstantCommand(wobbleGoalArm::setTurretMiddle, wobbleGoalArm),
                new DriveForwardCommand(drivetrain, -6),
                new TurnToCommand(drivetrain, 180),
                new InstantCommand(intake::autodropIntake, intake),
                new InstantCommand(intake::intake, intake),
                //go to rings
                new SplineCommand(drivetrain, new Vector2d(-18, -1), Math.toRadians(180)),
                new WaitCommand(1000),

                //drive to line and spin up wheels
                new InstantCommand(() -> shooterWheels.setShooterRPM(HG_SPEED), shooterWheels),
                new DriveForwardCommand(drivetrain, -20),


                //shoot
                new FeedRingsCommand(feeder, 5),
                new TurnToCommand(drivetrain, 180),
                new WaitCommand(500),

                //go back to rings
                new DriveForwardCommand(drivetrain, 40),
                new WaitCommand(500),
                //drive to line then shoot
                new DriveForwardCommand(drivetrain, -40),

                new FeedRingsCommand(feeder, 5),
                new InstantCommand(() -> shooterWheels.setShooterRPM(0), shooterWheels),
                new InstantCommand(intake::stop, intake),

                //park
                new DriveForwardCommand(drivetrain, -10)




        );
    }
}
