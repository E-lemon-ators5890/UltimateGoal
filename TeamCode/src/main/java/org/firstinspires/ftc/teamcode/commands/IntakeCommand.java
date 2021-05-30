package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;

public class IntakeCommand extends CommandBase {
    private Intake intake;
    ElapsedTime timer;
    public IntakeCommand(Intake intake) {
        timer = new ElapsedTime();
        this.intake = intake;
        this.addRequirements(intake);
    }

    @Override
    public void initialize() {
        timer.reset();

    }

    @Override
    public void execute() {
        intake.set(Math.abs(Math.sin(timer.seconds() * 2)));
    }
    
    @Override
    public void end(boolean interrupted) {
        intake.stop();
    }
}
