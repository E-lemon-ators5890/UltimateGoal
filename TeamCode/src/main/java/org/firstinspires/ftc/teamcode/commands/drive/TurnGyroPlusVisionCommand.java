package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

public class TurnGyroPlusVisionCommand extends CommandBase {

    private final Drivetrain drive;
    private final Vision vision;
    private double multiplier;
    public TurnGyroPlusVisionCommand(Drivetrain drive, Vision vision, double multiplier) {
        this.drive = drive;
        this.vision = vision;
        this.multiplier = multiplier;
        addRequirements(drive);
    }

    @Override
    public void initialize() {
        drive.turn(Math.toRadians(-vision.getHighGoalAngle()));
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            drive.stop();
        }
    }

    @Override
    public void execute() {
        drive.update();
    }

    @Override
    public boolean isFinished() {
        return Thread.currentThread().isInterrupted() || !drive.isBusy();
    }

}

