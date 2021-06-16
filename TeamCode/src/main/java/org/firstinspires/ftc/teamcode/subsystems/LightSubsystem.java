package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Util;

import java.util.logging.Level;

@Config
public class LightSubsystem extends SubsystemBase {
    private RevBlinkinLedDriver blinkinLedDriver;
    private RevBlinkinLedDriver.BlinkinPattern pattern;

    private Vision vision;
    private ShooterWheels shooterWheels;
    private WobbleGoalArm wobbleGoalArm;
    private final int SPEED_RANGE = 150;

    public static RevBlinkinLedDriver.BlinkinPattern DEFAULT_PATTERN = RevBlinkinLedDriver.BlinkinPattern.BLACK;
    public static double test = 0;
    public LightSubsystem(HardwareMap hw, Vision vision, ShooterWheels wheels, WobbleGoalArm wobbleGoalArm) {
        blinkinLedDriver = hw.get(RevBlinkinLedDriver.class, "blinkin");

        pattern = RevBlinkinLedDriver.BlinkinPattern.BLACK;

        this.vision = vision;
        this.shooterWheels = wheels;
        this.wobbleGoalArm = wobbleGoalArm;
    }

    /**
     If running Ring Detector -> Show patterns based on camera
     If Target Visible, show green if shooter on else blue violet
     If Shooter On (Target not visible), show FIRE_MEDIUM
     Else show SINELON_OCEAN_PALETTE
     */
    @Override
    public void periodic() {

        boolean isShooterOn = shooterWheels.getSetpoint() > 0 && Math.abs(shooterWheels.getSetpoint() - shooterWheels.getShooterRPM()) < SPEED_RANGE;
        if (vision.isRunningRingDetector()) {
            switch (vision.getCurrentStack()) {
                case FOUR:
                    pattern = RevBlinkinLedDriver.BlinkinPattern.YELLOW;
                    break;
                case ONE:
                    pattern = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE;
                    break;
                case ZERO:
                    pattern = RevBlinkinLedDriver.BlinkinPattern.CP1_LARSON_SCANNER;
                    break;
            }
        } else if (vision.isRunningHGDetector() && vision.isTargetVisible()) {
            if (isShooterOn) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN;
            } else {
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET;
            }
        } else if (isShooterOn) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.GOLD;
        } else {
            pattern = DEFAULT_PATTERN;
        }
        Util.logger(this, Level.INFO, "Pattern Name", pattern.name());
        blinkinLedDriver.setPattern(pattern);
    }
}
