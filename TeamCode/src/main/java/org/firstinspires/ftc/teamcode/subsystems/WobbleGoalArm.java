package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util;

import java.util.logging.Level;

@Config
public class WobbleGoalArm extends SubsystemBase {
    private Telemetry telemetry;
    private MotorEx arm;
    private TouchSensor homeSwitch;
    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.01, 0.0001, 0.003, 0);
    public static double ARM_OFFSET = -152;
    private PIDFController controller;
    private ServoEx claw, lazySusan;
    private boolean automatic;

    public static double CPR = 2786;
    public static double ARM_SPEED = 0.8;

    private double encoderOffset = 0;

    public WobbleGoalArm(MotorEx arm, ServoEx lazySusan, ServoEx claw, TouchSensor homeSensor, Telemetry tl) {
        this.arm = arm;
        this.arm.setDistancePerPulse(360/CPR);
        arm.setInverted(false);
        controller = new PIDFController(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d, pidfCoefficients.f,  getAngle(), getAngle());
        controller.setTolerance(10);

        this.claw = claw;
        this.lazySusan = lazySusan;
        this.homeSwitch = homeSensor;
        this.telemetry = tl;
        automatic = false;
        setOffset();
    }

    public void toggleAutomatic() {
        automatic = !automatic;
    }
    public boolean isAtHome() {
        return homeSwitch.isPressed();
    }
    public boolean isAutomatic() {
        return automatic;
    }

    @Override
    public void periodic() {
        if (automatic) {
           controller.setF(pidfCoefficients.f * Math.cos(Math.toRadians(controller.getSetPoint())));
           double output = controller.calculate(getAngle());
           arm.set(output);
        }

        Util.logger(this, telemetry, Level.INFO, "Wobble Claw Pos", claw.getPosition());
        Util.logger(this, telemetry, Level.INFO, "Wobble Angle", getAngle());
        Util.logger(this, telemetry, Level.INFO, "Wobble Goal", controller.getSetPoint());
        Util.logger(this, telemetry, Level.INFO, "Wobble Power", arm.get());
        Util.logger(this, telemetry, Level.INFO, "Home Position", isAtHome());
    }

    private double getEncoderDistance() {
        return arm.getDistance() - encoderOffset;
    }

    public void liftArmManual() {
        automatic = false;
        arm.set(-ARM_SPEED);
    }
    public void lowerArmManual() {
        automatic = false;
        arm.set(ARM_SPEED);
    }
    public void stopArm() {
        arm.stopMotor();
        controller.setSetPoint(getAngle());
        automatic = false;
    }

    public void setAutomatic(boolean auto) {
        this.automatic = auto;
    }

    public void resetEncoder() {
        encoderOffset = arm.getDistance();
    }

    public double getAngle() {
        return ARM_OFFSET + getEncoderDistance();
    }

    /************************************************************************************************/
    public void placeWobbleGoal() {
        // TODO CHNAGNE
        controller.setP(0.01);

        automatic = true;
        controller.setSetPoint(5);
    }
    public void liftWobbleGoal() {
        controller.setP(0.025);
        automatic = true;
        controller.setSetPoint(ARM_OFFSET + 5);
    }
    public void midWobbleGoal() {

        automatic = true;
        controller.setSetPoint(ARM_OFFSET + 40);
    }
    public void setWobbleGoal(double angle) {
        automatic = true;
        controller.setSetPoint(angle);
    }
    public boolean atTargetAngle() {
        return controller.atSetPoint();
    }

    public void setClawPosition(double position) {
        claw.setPosition(position);
    }
    public void openClaw() { setClawPosition(.75); }
    public void closeClaw() { setClawPosition(0.32); }

    public void setLazySusanPosition(double position) {
        lazySusan.setPosition(position);
    }
    public void setTurretLeft() {
        setLazySusanPosition(1);
    }
    public void setTurretRight() {
        setLazySusanPosition(0.143);
    }
    public void setTurretFarRight() {
        setLazySusanPosition(0);
    }
    public void setTurretMiddle() {
        setLazySusanPosition(0.566);
    }
    public void setTurretDiagonalRed() {
        setLazySusanPosition(0.8);
    }
    public void setTurretDiagonalBlue(){
        setLazySusanPosition(0.34);
    }

    public void setOffset() {
        resetEncoder();
        controller.setSetPoint(getAngle());
    }
}
