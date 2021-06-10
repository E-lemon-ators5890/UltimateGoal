package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util;

import java.util.logging.Level;
@Config
public class Intake extends SubsystemBase {

    public static double INTAKE_SPEED = 1;
    public static double OUTAKE_SPEED = -1;

    Telemetry telemetry;
    private MotorEx intake;
    private ServoEx servo;
    public Intake(MotorEx intake, ServoEx servo, Telemetry tl) {
        this.intake = intake;
        this.servo = servo;
        this.telemetry = tl;
    }

    @Override
    public void periodic() {
        Util.logger(this, telemetry, Level.INFO, "Current Intake Speed", intake.get());
    }

    public void set(double speed) {
        intake.set(speed);
    }
    public void dropIntake() {
        servo.setPosition(0.408);
    }
    public void autodropIntake() {
        servo.setPosition(0.38);
    }

    public void liftIntake() {
        servo.setPosition(0.8);
    }
    public void intake() {
        set(INTAKE_SPEED);
    }

    public void outtake() {
        set(OUTAKE_SPEED);
    }

    public void stop() {
        intake.stopMotor();
    }
}
