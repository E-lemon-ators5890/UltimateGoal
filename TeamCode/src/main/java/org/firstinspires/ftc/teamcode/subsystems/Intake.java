package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util;

import java.util.logging.Level;
@Config
public class Intake extends SubsystemBase {

    public static double INTAKE_SPEED = 1;
    public static double OUTAKE_SPEED = -1;

    Telemetry telemetry;
    TelemetryPacket packet;
    private MotorEx intake;

    public Intake(MotorEx intake, Telemetry tl, TelemetryPacket packet) {
        this.intake = intake;
        this.telemetry = tl;
        this.packet = packet;
    }

    @Override
    public void periodic() {
        Util.logger(this, telemetry, Level.INFO, "Current Intake Speed", intake.get(), packet);
    }

    private void set(double speed) {
        intake.set(speed);
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
