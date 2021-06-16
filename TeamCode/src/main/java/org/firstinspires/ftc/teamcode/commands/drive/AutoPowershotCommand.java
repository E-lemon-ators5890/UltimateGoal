package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.InstantCommand;
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

    public AutoPowershotCommand(Drivetrain drivetrain, ShooterFeeder feeder, ShooterWheels shooterWheels, Vision vision, double range, boolean red) {
        int multiplier = red ? -1 : 1;

        addCommands(
                new InstantCommand(() -> shooterWheels.setShooterRPM(2900), shooterWheels),
                new WaitCommand(1000),
                new InstantCommand(() -> {
                    vision.setOffset(85 * multiplier);
                }),
                new VisionCommand(drivetrain, vision, 0.05, range, 0.65),
                new FeedRingsCommand(feeder, 1),
                new WaitCommand(1000),
                new InstantCommand(() -> {
                    vision.setOffset(120 * multiplier);
                }),
                new TurnGyroPlusVisionCommand(drivetrain, vision),
                new WaitCommand(1000),
                new FeedRingsCommand(feeder, 1),
                new InstantCommand(() -> {
                    vision.setOffset(180 * multiplier);
                }),
                new TurnCommand(drivetrain, -8 * multiplier),
                new WaitCommand(1000),
                new FeedRingsCommand(feeder, 1),
                new InstantCommand(() -> {
                    vision.setOffset(-38);
                }),
                new InstantCommand(() -> shooterWheels.setShooterRPM(0))
        );
        addRequirements(drivetrain, feeder, shooterWheels, vision);
        // f(error) => output
        // f(error) = kP * error (-15 to 15)
        // output (-1 to 1)
        // (-0.5 to 0.5)
    }
}
