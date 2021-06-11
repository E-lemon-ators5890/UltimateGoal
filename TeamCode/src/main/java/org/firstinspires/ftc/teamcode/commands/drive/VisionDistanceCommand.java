package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;

import org.firstinspires.ftc.teamcode.commands.RunCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

public class VisionDistanceCommand extends ParallelCommandGroup {
    public VisionDistanceCommand(ShooterWheels wheels, Drivetrain drivetrain, Vision vision) {
        addCommands(
                new VisionCommand(drivetrain, vision, 15),
                new RunCommand(() -> {
                   if (vision.isTargetVisible() && vision.getHighGoalDistance() > 0) {
                       // TODO Fit Equation
                       try {
                           wheels.setShooterRPM(vision.lut.get(vision.getHighGoalDistance()));
                       } catch (Exception e) {
                           wheels.setShooterRPM(ShooterWheels.TARGET_SPEED);
                       }
                   } else {
                       wheels.setShooterRPM(ShooterWheels.TARGET_SPEED);
                   }
                }, vision, wheels)
        );
    }
}
