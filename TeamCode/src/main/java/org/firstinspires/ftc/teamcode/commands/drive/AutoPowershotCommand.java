package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelDeadlineGroup;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.RunCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.commands.shooter.FeedRingsCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

public class AutoPowershotCommand extends SequentialCommandGroup {

    ShooterWheels wheels;
    Vision vision;
    public AutoPowershotCommand(Drivetrain drivetrain, ShooterFeeder feeder, ShooterWheels shooterWheels, Vision vision, double range, boolean red) {
        int multiplier = red ? -1 : 1;

        this.wheels = shooterWheels;
        this.vision = vision;

        addCommands(
                new InstantCommand(() -> shooterWheels.setShooterRPM(2900), shooterWheels),
                new WaitCommand(1000),
                new InstantCommand(() -> {
                    vision.setOffset(75 * multiplier);
                }),
                new ParallelRaceGroup(
                        new VisionCommand(drivetrain, vision, 0.05, range, 0.65),
                        new WaitCommand(800)
                ),
                new FeedRingsCommand(feeder, 1),
                new WaitCommand(1000),
                new TurnCommand(drivetrain, -7 * multiplier),
                new WaitCommand(1000),
                new FeedRingsCommand(feeder, 1),
                new TurnCommand(drivetrain, -6 * multiplier),
                new WaitCommand(1000),
                new FeedRingsCommand(feeder, 1),
                new InstantCommand(() -> shooterWheels.setShooterRPM(0))
        );
        addRequirements(drivetrain, feeder, shooterWheels, vision);
        // f(error) => output
        // f(error) = kP * error (-15 to 15)
        // output (-1 to 1)
        // (-0.5 to 0.5)
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        wheels.setShooterRPM(0);
        vision.setOffset(-38);
    }
}
