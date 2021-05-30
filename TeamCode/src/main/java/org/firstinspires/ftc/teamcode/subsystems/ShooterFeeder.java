package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.ServoEx;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util;

import java.util.logging.Level;

@Config
public class ShooterFeeder extends SubsystemBase {

    public static double SERVO_POSITION_SHOOT = 0.75;
    public static double SERVO_POSITION_HOME = 0.88;

    private Telemetry telemetry;
    private TelemetryPacket packet;
    private ServoEx feedServo;

    public ShooterFeeder(ServoEx feedServo, Telemetry tl) {
        this.feedServo = feedServo;
        this.telemetry = tl;
        this.packet = packet;
    }

    @Override
    public void periodic() {
        Util.logger(this, telemetry, Level.INFO, "Feed Servo Position", feedServo.getPosition());
    }

    public void setFeedServo(double position) {
        feedServo.setPosition(position);
    }

    public void feedShooter() {
        setFeedServo(SERVO_POSITION_SHOOT);
    }
    public void retractFeed() {
        setFeedServo(SERVO_POSITION_HOME);
    }
}
