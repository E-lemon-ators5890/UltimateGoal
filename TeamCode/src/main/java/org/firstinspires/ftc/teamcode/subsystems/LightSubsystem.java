package org.firstinspires.ftc.teamcode.subsystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LightSubsystem extends SubsystemBase {
    private RevBlinkinLedDriver blinkinLedDriver;
    private RevBlinkinLedDriver.BlinkinPattern pattern;

    private Vision vision;
    private ShooterWheels shooterWheels;
    private WobbleGoalArm wobbleGoalArm;
    private final int SPEED_RANGE = 150;

    public LightSubsystem(HardwareMap hw, Vision vision, ShooterWheels wheels, WobbleGoalArm wobbleGoalArm) {
        blinkinLedDriver = hw.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;

        this.vision = vision;
        this.shooterWheels = wheels;
        this.wobbleGoalArm = wobbleGoalArm;
    }

    /**
     If running Ring Detector -> Show patterns based on camera
     If Target Visible, show green if shooter on else blue violet
     If Shooter On (Target not visible), show gold
     Else show confetti
     */
    @Override
    public void periodic() {

        boolean isShooterOn = shooterWheels.getSetpoint() > 0 && Math.abs(shooterWheels.getSetpoint() - shooterWheels.getShooterRPM()) < SPEED_RANGE;
        if (vision.isRunningRingDetector()) {
            switch (vision.getCurrentStack()) {
                case FOUR:
                    pattern = RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD;
                    break;
                case ONE:
                    pattern = RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE;
                    break;
                case ZERO:
                    pattern = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE;
                    break;
            }
        } else if (vision.isRunningHGDetector() && vision.isTargetVisible()) {
            if (isShooterOn) {
                pattern = RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN;
            } else {
                pattern = RevBlinkinLedDriver.BlinkinPattern.BLUE_VIOLET;
            }
        } else if (isShooterOn) {
            pattern = RevBlinkinLedDriver.BlinkinPattern.GOLD;
        } else {
            pattern = RevBlinkinLedDriver.BlinkinPattern.CONFETTI;
        }
        blinkinLedDriver.setPattern(pattern);
    }
}
