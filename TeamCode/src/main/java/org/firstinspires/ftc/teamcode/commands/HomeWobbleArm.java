package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

public class HomeWobbleArm extends CommandBase {
    private WobbleGoalArm arm;
    public HomeWobbleArm(WobbleGoalArm arm) {
        this.arm = arm;
        addRequirements(arm);
    }

    @Override
    public void initialize() {
        arm.liftArmManual();
    }

    @Override
    public boolean isFinished() {
       return arm.isAtHome();
    }

    @Override
    public void end(boolean interrupted) {
        arm.stopArm();
        arm.setOffset();
        arm.setAutomatic(true);
    }
}
