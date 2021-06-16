package org.firstinspires.ftc.teamcode.commands.shooter;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;

public class SmartFeedRingsCommand extends SequentialCommandGroup {
    private final int RANGE = 300;

    public SmartFeedRingsCommand(ShooterFeeder feeder, ShooterWheels wheels, int numRings, int timeout) {
        for (int i = 0; i < numRings; i++) {
            addCommands(
                    new ConditionalCommand(
                            new SequentialCommandGroup(
                                    new WaitCommand(timeout),
                                    new InstantCommand(feeder::feedShooter),
                                    new WaitCommand(timeout),
                                    new InstantCommand(feeder::retractFeed)
                            ),
                            new InstantCommand(),
                            () -> wheels.getSetpoint() > 0 && Math.abs(wheels.getSetpoint() - wheels.getShooterRPM()) < RANGE
                    )

            );
        }
        addRequirements(feeder, wheels);

    }
    public SmartFeedRingsCommand(ShooterFeeder feeder, ShooterWheels wheels, int numRings) {
        this(feeder, wheels, numRings, 50);
    }
}
