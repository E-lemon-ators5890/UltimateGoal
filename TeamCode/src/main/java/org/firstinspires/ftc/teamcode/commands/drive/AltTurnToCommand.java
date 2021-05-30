package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.logging.Level;

public class AltTurnToCommand extends CommandBase {

    private final Drivetrain drive;
    private final double angle;
    private PIDController turningController;
    private double possibleRange;

    public static double P = .005;
    public static double I = 0.0000001;
    public static double D = 0.00001;
    public static double MAX_SPEED = 0.6;
    public static double TOLERANCE = 2;

    public AltTurnToCommand(Drivetrain drive, double angleDegrees) {
        this.drive = drive;
        this.angle = angleDegrees;

        turningController = new PIDController(P, I, D);
        // Setpoint: 90 Current POsition: 0
        // Error = setpoint - pos
        // error = 90
        // output (-1 and 1): kP * Error
        // output = 0.001 * 90

        turningController.setTolerance(2);
        addRequirements(drive);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }

    @Override
    public void execute() {
        double output = turningController.calculate(drive.getHeading(), angle);
        if (output < -MAX_SPEED)
            output = -MAX_SPEED;
        else if (output > MAX_SPEED)
            output = MAX_SPEED;

        drive.arcadeDrive(0, output);
    }

    @Override
    public boolean isFinished() {
        return turningController.atSetPoint();
    }

}

