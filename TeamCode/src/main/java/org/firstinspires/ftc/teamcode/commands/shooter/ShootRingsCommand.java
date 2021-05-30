package org.firstinspires.ftc.teamcode.commands.shooter;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;

public class ShootRingsCommand extends SequentialCommandGroup {
    public ShootRingsCommand(ShooterWheels shooter, ShooterFeeder feeder, double rpm, int numRings) {
        this(shooter, feeder, rpm, numRings, 500);
    }
    public ShootRingsCommand(ShooterWheels shooter, ShooterFeeder feeder, double rpm, int numRings, int timeout) {
        addCommands(
                new InstantCommand(() -> shooter.setShooterRPM(rpm)),
                new WaitUntilCommand(shooter::atSetpoint),
                new FeedRingsCommand(feeder, numRings, timeout),
                new WaitCommand(timeout),
                new InstantCommand(() -> shooter.setShooterRPM(0))
        );
    }
}
