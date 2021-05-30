package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.control.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.followers.RamseteFollower;
import com.acmerobotics.roadrunner.followers.TrajectoryFollower;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.kinematics.Kinematics;
import com.acmerobotics.roadrunner.kinematics.TankKinematics;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.util.NanoClock;
import com.arcrobotics.ftclib.controller.PIDFController;

import org.jetbrains.annotations.NotNull;
import org.jetbrains.annotations.Nullable;

import java.util.Arrays;
import java.util.List;

import static com.acmerobotics.roadrunner.kinematics.TankKinematics.robotToWheelVelocities;
import static java.lang.Math.cos;
import static java.lang.Math.sin;
import static java.lang.Math.sqrt;

public class RamseteFollowerPID extends TrajectoryFollower {
    private static final double INCHES_TO_METERS = 0.0254;
    PIDFController leftVeloPID, rightVeloPID;
    double b, zeta;
    List<Double> prevSpeeds;
    Pose2d lastError;

    double prevTime;
    public RamseteFollowerPID(double b, double zeta, @NotNull Pose2d admissibleError, double timeout) {
        super(new Pose2d(admissibleError.getX() * INCHES_TO_METERS, admissibleError.getY() * INCHES_TO_METERS, admissibleError.getHeading()), timeout, NanoClock.system());
        this.b = b;
        this.zeta = zeta;
        lastError = new Pose2d();
    }

    @Override
    public void followTrajectory(@NotNull Trajectory trajectory) {
        prevTime = -1;
        Pose2d intialState = trajectory.velocity(0);
        prevSpeeds = TankKinematics.robotToWheelVelocities(new Pose2d(intialState.getX() * INCHES_TO_METERS, intialState.getY() * INCHES_TO_METERS, intialState.getHeading()), DriveConstants.TRACK_WIDTH);
        leftVeloPID.reset();
        rightVeloPID.reset();
        super.followTrajectory(trajectory);
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
        double dt = t - prevTime;
        
        if (prevTime < 0) {
            prevTime = t;
            return new DriveSignal(new Pose2d(0, 0, 0));
        }

        
        Pose2d targetInchesPose = trajectory.get(t);
        Pose2d targetInchesVelocity = trajectory.velocity(t);
        Pose2d chassisSpeeds = calculateChassisSpeedsFromMeters(new Pose2d(targetInchesPose.getX() * INCHES_TO_METERS, targetInchesPose.getY() * INCHES_TO_METERS, targetInchesPose.getHeading()),
                new Pose2d(targetInchesVelocity.getX() * INCHES_TO_METERS, targetInchesVelocity.getY() * INCHES_TO_METERS, targetInchesVelocity.getHeading()),new Pose2d(currentPose.getX() * INCHES_TO_METERS, currentPose.getY() * INCHES_TO_METERS, currentPose.getHeading()), new Pose2d(currentRobotVel.getX() * INCHES_TO_METERS, currentRobotVel.getY() * INCHES_TO_METERS, currentRobotVel.getHeading()));
        List<Double> targetWheelSpeeds = TankKinematics.robotToWheelVelocities(chassisSpeeds, DriveConstants.TRACK_WIDTH);
        double leftSpeedSetpoint = targetWheelSpeeds.get(0);
        double rightSpeedSetpoint = targetWheelSpeeds.get(1);

        double leftFeedforward = Kinematics.calculateMotorFeedforward(leftSpeedSetpoint, (leftSpeedSetpoint - prevSpeeds.get(0)) / dt, DriveConstants.kV, DriveConstants.kA, DriveConstants.kStatic);

        return null;
    }

    private Pose2d calculateChassisSpeedsFromMeters(Pose2d targetPose, Pose2d targetVel, Pose2d currentPose, Pose2d currentVel) {

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
        return new Pose2d(v, 0, omega);
    }

    private static double sinc(double x) {
        if (Math.abs(x) < 1e-9) {
            return 1.0 - 1.0 / 6.0 * x * x;
        } else {
            return sin(x) / x;
        }
    }
}
