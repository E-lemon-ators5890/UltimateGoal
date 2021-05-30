package org.firstinspires.ftc.teamcode.commands.drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.ConditionalCommand;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

@Config
public class TurnToGoalCommand extends ConditionalCommand {

    public TurnToGoalCommand(Drivetrain drivetrain, Vision vision, double defaultTurnToAngle) {
        super(new VisionCommand(drivetrain, vision, 20), new TurnToCommand(drivetrain, defaultTurnToAngle), () -> vision.isTargetVisible());
    }

}
