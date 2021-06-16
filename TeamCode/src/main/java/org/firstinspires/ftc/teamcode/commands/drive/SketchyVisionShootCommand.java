package org.firstinspires.ftc.teamcode.commands.drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelRaceGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.controller.PIDController;

import org.firstinspires.ftc.teamcode.Util;
import org.firstinspires.ftc.teamcode.commands.shooter.FeedRingsCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

import java.util.logging.Level;

@Config
public class SketchyVisionShootCommand extends SequentialCommandGroup {
    private Vision vision;
    private Drivetrain drivetrain;
    private ShooterWheels wheels;

    private PIDController turningController;
    private double possibleRange;

    public static double VISION_P = 0.03;
    public static double VISION_I = 0.000001;
    public static double VISION_D = 0.00001;
    public static double MAX_SPEED = 0.42;

    public SketchyVisionShootCommand(Drivetrain drivetrain, ShooterWheels wheels, ShooterFeeder feeder, Vision vision, double range) {
        this.wheels = wheels;

        addCommands(
                new InstantCommand(() -> wheels.setShooterRPM(ShooterWheels.TARGET_SPEED), wheels),
                new ParallelRaceGroup(
                        new VisionCommand(drivetrain, vision, range),
                        new WaitCommand(1200)
                        ),
                new WaitCommand(500),
                new FeedRingsCommand(feeder, 3),
                new InstantCommand(() -> wheels.setShooterRPM(0), wheels)
        );

    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
        wheels.setShooterRPM(0);
    }
}
