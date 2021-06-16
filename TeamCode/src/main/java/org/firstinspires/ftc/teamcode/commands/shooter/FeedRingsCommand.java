package org.firstinspires.ftc.teamcode.commands.shooter;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;

public class FeedRingsCommand extends SequentialCommandGroup {
    public FeedRingsCommand(ShooterFeeder feeder, int numRings, int timeout) {
        for (int i = 0; i < numRings; i++) {
            addCommands(
                new WaitCommand(timeout),
                new InstantCommand(feeder::feedShooter, feeder),
                new WaitCommand(timeout),
                new InstantCommand(feeder::retractFeed, feeder)

            );
        }
        addRequirements(feeder);
    }
    public FeedRingsCommand(ShooterFeeder feeder, int numRings) {
        this(feeder, numRings, 50);
    }
}
