package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.TankKinematics;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.List;

import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

public class RamseteFollowerMeters extends TrajectoryFollower {
    private static final double INCHES_TO_METERS = 0.0254;
    double b, zeta;
    Pose2d lastError;

    public RamseteFollowerMeters(double b, double zeta, @NotNull Pose2d admissibleError, double timeout) {
        super(new Pose2d(admissibleError.getX() * INCHES_TO_METERS, admissibleError.getY() * INCHES_TO_METERS, admissibleError.getHeading()), timeout, NanoClock.system());
        this.b = b;
        this.zeta = zeta;
        lastError = new Pose2d();
    }


    @NotNull
    @Override
    public Pose2d getLastError() {
        return lastError;
    }

    @Override
    protected void setLastError(@NotNull Pose2d pose2d) {
        this.lastError = pose2d;
    }

    @NotNull
    @Override
    protected DriveSignal internalUpdate(@NotNull Pose2d currentPose, @Nullable Pose2d currentRobotVel) {
        double t = elapsedTime();

        Pose2d targetInchesPose = trajectory.get(t);
        Pose2d targetInchesVelocity = trajectory.velocity(t);
        Pose2d chassisSpeeds = calculateChassisSpeedsFromMetersToInches(new Pose2d(targetInchesPose.getX() * INCHES_TO_METERS, targetInchesPose.getY() * INCHES_TO_METERS, targetInchesPose.getHeading()),
                new Pose2d(targetInchesVelocity.getX() * INCHES_TO_METERS, targetInchesVelocity.getY() * INCHES_TO_METERS, targetInchesVelocity.getHeading()),new Pose2d(currentPose.getX() * INCHES_TO_METERS, currentPose.getY() * INCHES_TO_METERS, currentPose.getHeading()));
        return new DriveSignal(chassisSpeeds);
    }

    private Pose2d calculateChassisSpeedsFromMetersToInches(Pose2d targetPose, Pose2d targetVel, Pose2d currentPose) {

        Pose2d targetRobotVel = Kinematics.fieldToRobotVelocity(targetPose, targetVel);

        double targetV = targetRobotVel.getX();
        double targetOmega = targetRobotVel.getHeading();

        Pose2d error = Kinematics.calculatePoseError(targetPose, currentPose);

        double k1 = 2 * zeta * sqrt(targetOmega * targetOmega + b * targetV * targetV);
        double k3 = k1;
        double k2 = b;

        double v = targetV * cos(error.getHeading()) +
                k1 * (cos(currentPose.getHeading()) * error.getX() + sin(currentPose.getHeading()) * error.getY());
        double omega = targetOmega + k2 * targetV * sinc(error.getHeading()) *
                (cos(currentPose.getHeading()) * error.getY() - sin(currentPose.getHeading()) * error.getX()) +
                k3 * error.getHeading();

        lastError = Kinematics.calculatePoseError(targetPose, currentPose);
        return new Pose2d(v / INCHES_TO_METERS, 0, omega);

    }

    private static double sinc(double x) {
        if (Math.abs(x) < 1e-9) {
            return 1.0 - 1.0 / 6.0 * x * x;
        } else {
            return sin(x) / x;
        }
    }
}
