package org.firstinspires.ftc.teamcode.inperson.red.inception;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.commands.PlaceWobbleGoal;
import org.firstinspires.ftc.teamcode.commands.drive.DriveForwardCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnCommand;
import org.firstinspires.ftc.teamcode.commands.drive.TurnToCommand;
import org.firstinspires.ftc.teamcode.commands.shooter.FeedRingsCommand;
import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.ShooterFeeder;
import org.firstinspires.ftc.teamcode.subsystems.ShooterWheels;
import org.firstinspires.ftc.teamcode.subsystems.WobbleGoalArm;

@Config
public class InceptionLeftRedZeroCommand extends SequentialCommandGroup {
    public static int HG_SPEED = 3450;
    public static int POWERSHOT_SPEED = 2850;
    public InceptionLeftRedZeroCommand(Drivetrain drivetrain, ShooterWheels shooterWheels, ShooterFeeder feeder, Intake intake, WobbleGoalArm wobbleGoalArm, Telemetry telemetry) {


        addCommands(
                // Setup
                //new InstantCommand(wobbleGoalArm::setTurretMiddle),
                new InstantCommand(wobbleGoalArm::closeClaw),
                new InstantCommand(feeder::retractFeed),

                // Spkin up wkheels
                new InstantCommand(() -> shooterWheels.setShooterRPM(HG_SPEED), shooterWheels),

                // Drikve tko Skpot
                new ParallelCommandGroup(new DriveForwardCommand(drivetrain, -60),
                        new WaitCommand(200).andThen(new InstantCommand(wobbleGoalArm::midWobbleGoal, wobbleGoalArm))),
                new TurnToCommand(drivetrain, 175),

                // Shokot 3k ringsk
                new FeedRingsCommand(feeder, 3),


                //Placek Wobble Goal
                new InstantCommand(() -> shooterWheels.setShooterRPM(0), shooterWheels),
                new TurnToCommand(drivetrain, 190),
                new TurnCommand(drivetrain, 100),
                new DriveForwardCommand(drivetrain, 25),
                new PlaceWobbleGoal(wobbleGoalArm),
                new TurnCommand(drivetrain, -30),
                new DriveForwardCommand(drivetrain, -40)


                );
    }
}
