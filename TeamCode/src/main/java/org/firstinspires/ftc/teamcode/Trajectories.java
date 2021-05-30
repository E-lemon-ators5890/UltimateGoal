package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.ProfileAccelerationConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TankVelocityConstraint;

import java.util.Arrays;
import java.util.Vector;

import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ACCEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_ANG_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.MAX_VEL;
import static org.firstinspires.ftc.teamcode.drive.DriveConstants.TRACK_WIDTH;
public class Trajectories {

    public static MinVelocityConstraint velConstraint = new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(MAX_ANG_VEL),
                new TankVelocityConstraint(MAX_VEL, TRACK_WIDTH)
        ));
    public static MinVelocityConstraint slowConstraint = new MinVelocityConstraint(Arrays.asList(
            new AngularVelocityConstraint(MAX_ANG_VEL),
            new TankVelocityConstraint(30, TRACK_WIDTH)
    ));




    public static ProfileAccelerationConstraint accelConstraint = new ProfileAccelerationConstraint(MAX_ACCEL);
    public static ProfileAccelerationConstraint slowAccelConstraint = new ProfileAccelerationConstraint(MAX_ACCEL);

    @Config
    public static class BlueLeftTape {
        public static Pose2d startPose = new Pose2d(-62.5, 52, Math.toRadians(180));

        public static double wobbleGoalX = 66;
        public static double wobbleGoalY = 53;

        public static double highGoalX = -3;
        public static double highGoalY = 39;
        public static double intakeFirst = 36;
        public static double intakeDistance = 12;
        public static double shootMoreDistance = 24;
        public static double wobbleDistance = 9;
        public static double wobbleAngle = 226;



    }
    @Config
    public static class BlueMid {
        public static Pose2d startPose = new Pose2d(-62.5, 52, Math.toRadians(180));
        public static double shootDistance = 60;
        public static double wobbleGoalSquareDistance = 48;

        public static double ringX = -30;
        public static double ringY = 32;
        public static double intakeFirst = 36;
        public static double intakeDistance = 12;
        public static double shootMoreDistance = 24;
        public static double wobbleDistance = 9.25;
        public static double wobbleAngle = 211;

    }
    @Config
    public static class BlueCloseTape {
        public static Pose2d startPose = new Pose2d(-62.5, 52, Math.toRadians(180));

        public static double wobbleGoalSquareDistance = 84;
        public static double wobbleGoalX = 20;

        public static double wobbleGoalY = 30;

        public static double highGoalX = -3;
        public static double highGoalY = 38;
        public static double intakeFirst = 36;
        public static double intakeDistance = 12;
        public static double shootMoreDistance = 24;
        public static double wobbleDistance = 26;
        public static double wobbleAngle = 195;






    }

}
