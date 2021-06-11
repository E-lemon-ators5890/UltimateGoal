package org.firstinspires.ftc.teamcode.inperson.red.spicy;

//untested

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

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

public class SpicyRedOneCommand extends SequentialCommandGroup {
    public SpicyRedOneCommand(Drivetrain drivetrain, ShooterWheels shooterWheels, ShooterFeeder feeder, Intake intake, WobbleGoalArm wobbleGoalArm, Vision vision, Telemetry telemetry) {
        final int HG_SPEED = 3450;
        final int POWERSHOT_SPEED = 3000;

        addCommands(
                new RightRedShootingSequence(drivetrain, shooterWheels, feeder, intake, wobbleGoalArm, telemetry),

                //Place Wobble Goal
                new TurnToCommand(drivetrain, 170),
                new InstantCommand(() -> shooterWheels.setShooterRPM(0), shooterWheels),
                new DriveForwardCommand(drivetrain, -45),
                //new InstantCommand(wobbleGoalArm::setTurretLeft,wobbleGoalArm),
                new WaitCommand(500),
                new PlaceWobbleGoal(wobbleGoalArm),
                new WaitCommand(500),
                new InstantCommand(wobbleGoalArm::openClaw, wobbleGoalArm),
                new InstantCommand(wobbleGoalArm::liftWobbleGoal, wobbleGoalArm),
                //new InstantCommand(wobbleGoalArm::setTurretMiddle, wobbleGoalArm),
                new SplineCommand(drivetrain, new Vector2d(15.5, -10), Math.toRadians(0)),
                // new DriveForwardCommand(drivetrain, 30),
                new TurnToCommand(drivetrain,0, true)




        );
    }
}
