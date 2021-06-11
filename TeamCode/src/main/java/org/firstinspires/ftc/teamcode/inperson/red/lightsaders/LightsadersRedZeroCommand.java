package org.firstinspires.ftc.teamcode.inperson.red.lightsaders;

//tested with them

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.PlaceWobbleGoal;
import org.firstinspires.ftc.teamcode.commands.drive.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.drive.SplineCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnToGoalCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.FeedRingsCommand;
import org.firstinspires.ftc.teamcode.inperson.red.RightRedShootingSequence;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;
import org.firstinspires.ftc.teamcode.subsystems.Vision;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

import static org.firstinspires.ftc.teamcode.commands.drive.TurnToCommand.redRightAngle;
import static org.firstinspires.ftc.teamcode.commands.drive.TurnToCommand.redLeftAngle;
import static org.firstinspires.ftc.teamcode.commands.drive.TurnToCommand.blueRightAngle;
import static org.firstinspires.ftc.teamcode.commands.drive.TurnToCommand.blueLeftAngle;

public class LightsadersRedZeroCommand extends SequentialCommandGroup {
    public LightsadersRedZeroCommand(Drivetrain drivetrain, ShooterWheels shooterWheels, ShooterFeeder feeder, Intake intake, WobbleGoalArm wobbleGoalArm, Vision vision, Telemetry telemetry) {
        final int HG_SPEED = 3450;
        final int POWERSHOT_SPEED = 3000;

        addCommands(
                new RightRedShootingSequence(drivetrain, shooterWheels, feeder, intake, wobbleGoalArm, telemetry),

                new TurnToCommand(drivetrain, 170),
                new DriveForwardCommand(drivetrain, -15),
                new TurnCommand(drivetrain,90),
                new DriveForwardCommand(drivetrain, -3),
                new PlaceWobbleGoal(wobbleGoalArm),
                new DriveForwardCommand(drivetrain, -3)

        );
    }
}
