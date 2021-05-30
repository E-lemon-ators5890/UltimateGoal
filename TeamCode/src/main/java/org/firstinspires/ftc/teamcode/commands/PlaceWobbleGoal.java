package org.firstinspires.ftc.teamcode.commands;

import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

public class PlaceWobbleGoal extends SequentialCommandGroup {
    private WobbleGoalArm wobbleGoal;

    public PlaceWobbleGoal(WobbleGoalArm wobbleGoal) {
        addCommands(
                new InstantCommand(wobbleGoal::placeWobbleGoal, wobbleGoal),
                new WaitUntilCommand(wobbleGoal::atTargetAngle).raceWith(new WaitCommand(1000)),
                new InstantCommand(wobbleGoal::stopArm, wobbleGoal),
                new WaitCommand(500),
                new InstantCommand(() -> wobbleGoal.openClaw()),
                new WaitCommand(500),
                new InstantCommand(wobbleGoal::liftWobbleGoal, wobbleGoal),
                new WaitCommand(300)
        );
    }
}
