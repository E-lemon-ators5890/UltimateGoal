package org.firstinspires.ftc.teamcode.commands;

import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Trajectories;
import org.firstinspires.ftc.teamcode.commands.drive.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.drive.SplineCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.FeedRingsCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.ShootRingsCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

import static org.firstinspires.ftc.teamcode.Trajectories.BlueLeftTape.highGoalX;
import static org.firstinspires.ftc.teamcode.Trajectories.BlueLeftTape.highGoalY;
import static org.firstinspires.ftc.teamcode.Trajectories.BlueLeftTape.intakeDistance;
import static org.firstinspires.ftc.teamcode.Trajectories.BlueLeftTape.intakeFirst;
import static org.firstinspires.ftc.teamcode.Trajectories.BlueLeftTape.shootMoreDistance;
import static org.firstinspires.ftc.teamcode.Trajectories.BlueLeftTape.wobbleGoalX;
import static org.firstinspires.ftc.teamcode.Trajectories.BlueLeftTape.wobbleGoalY;

public class TeleopPowershot extends SequentialCommandGroup {

    public TeleopPowershot(Drivetrain drivetrain, ShooterWheels shooterWheels, ShooterFeeder feeder, Telemetry telemetry) {
        addCommands(
                // Starts shooter
                new InstantCommand(() -> shooterWheels.setShooterRPM(2750), shooterWheels),
                new InstantCommand(feeder::retractFeed, feeder),
                new WaitUntilCommand(shooterWheels::atSetpoint).raceWith(new WaitCommand(800)),

                // Shoots/turn repeat
                new FeedRingsCommand(feeder, 1),
                    new TurnCommand(drivetrain, 6),
                new WaitCommand(200),
                new FeedRingsCommand(feeder, 1),
                new TurnCommand(drivetrain, 7),
                new WaitCommand(200),
                new FeedRingsCommand(feeder, 1),
                new InstantCommand(shooterWheels::stopShooter, shooterWheels)
                );
    }
}
