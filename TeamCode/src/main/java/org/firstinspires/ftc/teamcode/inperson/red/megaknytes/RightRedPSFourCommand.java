package org.firstinspires.ftc.teamcode.inperson.red.megaknytes;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.PlaceWobbleGoal;
import org.firstinspires.ftc.teamcode.commands.drive.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.FeedRingsCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

public class RightRedPSFourCommand extends SequentialCommandGroup {
    public RightRedPSFourCommand(Drivetrain drivetrain, ShooterWheels shooterWheels, ShooterFeeder feeder, Intake intake, WobbleGoalArm wobbleGoalArm, Telemetry telemetry) {
        final int HG_SPEED = 3450;
        final int POWERSHOT_SPEED = 3000;

        addCommands(
                // Setup
                //new InstantCommand(wobbleGoalArm::setTurretMiddle),
                new InstantCommand(wobbleGoalArm::closeClaw),
                new InstantCommand(feeder::retractFeed),
                new WaitCommand(10000),

                // Spin up wheels
                new InstantCommand(() -> shooterWheels.setShooterRPM(HG_SPEED), shooterWheels),

                // Drive to Spot
                new ParallelCommandGroup(new DriveForwardCommand(drivetrain, -60),
                        new WaitCommand(200).andThen(new InstantCommand(wobbleGoalArm::midWobbleGoal, wobbleGoalArm))),
                new TurnToCommand(drivetrain, 193),

                // Shoot 2 rings
                new FeedRingsCommand(feeder, 2),
                // Shoot ring at the Powershots
                new InstantCommand(() -> shooterWheels.setShooterRPM(POWERSHOT_SPEED), shooterWheels),
                new WaitCommand(500),
                new TurnToCommand(drivetrain, 204),
                new FeedRingsCommand(feeder, 2),
                //Place Wobble Goal
                new TurnToCommand(drivetrain, 170),
                new InstantCommand(() -> shooterWheels.setShooterRPM(0), shooterWheels),
                new DriveForwardCommand(drivetrain, -60),
                //new InstantCommand(wobbleGoalArm::setTurretFarRight,wobbleGoalArm),
                new WaitCommand(500),
                new PlaceWobbleGoal(wobbleGoalArm),
                new WaitCommand(500),
                new InstantCommand(wobbleGoalArm::openClaw, wobbleGoalArm),
                new InstantCommand(wobbleGoalArm::liftWobbleGoal, wobbleGoalArm),
                //new InstantCommand(wobbleGoalArm::setTurretMiddle, wobbleGoalArm),
                new DriveForwardCommand(drivetrain, 50)










        );
    }
}

