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
    public static PIDFCoefficients pidfCoefficients = new PIDFCoefficients(0.01, 0.000, 0.00, 0);
    public static double ARM_OFFSET = -150.6346015793252;
    private PIDFController controller;
    private ServoEx leftClaw, rightClaw;
    private boolean automatic;

    public static double CPR = 2786;
    public static double ARM_SPEED = 0.8;

    private double encoderOffset = 0;

    public WobbleGoalArm(MotorEx arm, ServoEx leftClaw, ServoEx rightClaw, TouchSensor homeSensor, Telemetry tl) {
        this.arm = arm;
        this.arm.setDistancePerPulse(360/CPR);
        arm.setInverted(false);
        controller = new PIDFController(pidfCoefficients.p, pidfCoefficients.i, pidfCoefficients.d, pidfCoefficients.f,  getAngle(), getAngle());
        controller.setTolerance(5);

        this.leftClaw = leftClaw;
        this.rightClaw = rightClaw;
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
            double output = -controller.calculate(getAngle());
            arm.set(output);
        }

        Util.logger(this, telemetry, Level.INFO, "Left Wobble Claw Pos", leftClaw.getPosition());
        Util.logger(this, telemetry, Level.INFO, "Right Wobble Claw Pos", rightClaw.getPosition());
        Util.logger(this, telemetry, Level.INFO, "Wobble Angle", getAngle());
        Util.logger(this, telemetry, Level.INFO, "Absolute Wobble Angle", getEncoderDistance());

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
        return ARM_OFFSET - getEncoderDistance();
    }

    /************************************************************************************************/
    public void placeWobbleGoal() {
        // TODO CHANGE

        automatic = true;
        controller.setSetPoint(0);
    }
    public void liftWobbleGoal() {

        automatic = true;
        controller.setSetPoint(ARM_OFFSET + 2);
    }
    public void midWobbleGoal() {

        automatic = true;
        controller.setSetPoint(ARM_OFFSET + 45);
    }
    public void setWobbleGoal(double angle) {
        automatic = true;
        controller.setSetPoint(angle);
    }
    public boolean atTargetAngle() {
        return controller.atSetPoint();
    }

    public void setClawPosition(double leftPosition, double rightPosition) {
        leftClaw.setPosition(leftPosition);
        rightClaw.setPosition(rightPosition);
    }

    public void closeClaw() { setClawPosition(0.5250 , 0.92); }
    public void openClaw() { setClawPosition(0.9125, 0.54); }

    public void setOffset() {
        resetEncoder();
        controller.setSetPoint(getAngle());
    }
}