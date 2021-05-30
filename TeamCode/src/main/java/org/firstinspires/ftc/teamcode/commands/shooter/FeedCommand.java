package org.firstinspires.ftc.teamcode.commands.shooter;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;

public class FeedCommand extends CommandBase {
    private ShooterFeeder feeder;
    private double position;

    public FeedCommand(ShooterFeeder feeder, double position) {
        this.feeder = feeder;
        this.position = position;
        
        this.addRequirements(feeder);
    }

    @Override
    public void initialize() {
        feeder.setFeedServo(position);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
