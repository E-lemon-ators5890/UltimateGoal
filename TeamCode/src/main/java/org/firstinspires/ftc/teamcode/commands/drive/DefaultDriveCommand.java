package org.firstinspires.ftc.teamcode.commands.drive;

import com.arcrobotics.ftclib.command.CommandBase;
import com.arcrobotics.ftclib.command.Subsystem;
import com.arcrobotics.ftclib.gamepad.GamepadEx;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;

import java.util.Set;
import java.util.function.DoubleSupplier;

public class DefaultDriveCommand extends CommandBase {
    private Drivetrain drive;
    private GamepadEx driverGamepad;

    protected double multiplier;

    public DefaultDriveCommand(Drivetrain drive, GamepadEx driverGamepad) {

        this.drive = drive;
        this.driverGamepad = driverGamepad;

        this.multiplier = 1;
        addRequirements(this.drive);
    }

    @Override
    public void execute() {
        // Arcade Drive
        // drive.arcadeDrive(driverGamepad.getLeftY(), driverGamepad.getRightX());

        // Tank Drive
        //drive.arcadeDrive(driverGamepad.getLeftY() * multiplier, driverGamepad.getRightX() * multiplier);
        drive.tankDrive(driverGamepad.getLeftY() * multiplier, driverGamepad.getRightY() * multiplier);
    }

    @Override
    public void end(boolean interrupted) {
        drive.stop();
    }
}
