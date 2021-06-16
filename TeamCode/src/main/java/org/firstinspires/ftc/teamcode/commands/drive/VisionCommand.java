package org.firstinspires.ftc.teamcode.commands.drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Util;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import java.util.logging.Level;

import kotlin.Experimental;

@Config
public class VisionCommand extends CommandBase {
    private Vision vision;
    private Drivetrain drivetrain;
    private PIDController turningController;
    private double possibleRange;

    public static double VISION_P = 0.03;
    public static double VISION_I = 0.000001;
    public static double VISION_D = 0.00001;
    public static double MAX_SPEED = 0.42;
    public static double TOLERANCE = 0.75;

    public VisionCommand(Drivetrain drivetrain, Vision vision, double range) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.possibleRange = range;

        turningController = new PIDController(VISION_P, VISION_I, VISION_D);
        // f(error) => output
        // f(error) = kP * error (-15 to 15)
        // output (-1 to 1)
        // (-0.5 to 0.5)
    }
    public VisionCommand(Drivetrain drivetrain, Vision vision, double p, double range, double tolerance) {
        this.drivetrain = drivetrain;
        this.vision = vision;
        this.possibleRange = range;


        turningController = new PIDController(p, VISION_I, VISION_D);
        TOLERANCE = tolerance;
        addRequirements(drivetrain, vision);
        // f(error) => output
        // f(error) = kP * error (-15 to 15)
        // output (-1 to 1)
        // (-0.5 to 0.5)
    }

    @Override
    public void initialize() {
        Util.logger(this, Level.WARNING, "Target visible", vision.isTargetVisible());
    }

    @Override
    public void execute() {
        // Powershot offset

        double output = turningController.calculate(-vision.getHighGoalAngle(), 0);
        if (output < -MAX_SPEED)
            output = -MAX_SPEED;
        else if (output > MAX_SPEED)
            output = MAX_SPEED;

        drivetrain.arcadeDrive(0, output);
    }

    @Override
    public boolean isFinished() {
        return Math.abs(vision.getHighGoalAngle()) > possibleRange || Math.abs(vision.getHighGoalAngle()) < TOLERANCE;
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.stop();
        Util.logger(this, Level.INFO, "Angles" ,-vision.getHighGoalAngle());
    }
}
