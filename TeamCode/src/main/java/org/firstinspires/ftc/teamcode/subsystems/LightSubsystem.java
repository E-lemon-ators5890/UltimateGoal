package org.firstinspires.ftc.teamcode.subsystems;


import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class LightSubsystem extends SubsystemBase {
    RevBlinkinLedDriver blinkinLedDriver;
    RevBlinkinLedDriver.BlinkinPattern pattern;

    public LightSubsystem(HardwareMap hw) {
        blinkinLedDriver = hw.get(RevBlinkinLedDriver.class, "blinkin");
        pattern = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
    }

    @Override
    public void periodic() {
       turnOnLights(pattern);
    }

    public void setPattern(RevBlinkinLedDriver.BlinkinPattern pattern) {
        this.pattern = pattern;
    }

    public void setZeroStack() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE);
    }

    public void setOneStack() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE);
    }

    public void setFourStack() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD);
    }

    public void setTeleop() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.CONFETTI);
    }

    public void setAutoAim() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.LAWN_GREEN);
    }

    public void stopLight() {
        setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
    }

    public void turnOnLights(RevBlinkinLedDriver.BlinkinPattern p) {
        blinkinLedDriver.setPattern(p);
    }

}
