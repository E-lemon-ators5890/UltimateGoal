package org.firstinspires.ftc.teamcode;

import androidx.annotation.NonNull;

import com.arcrobotics.ftclib.command.Command;
import com.arcrobotics.ftclib.command.button.Button;
import com.arcrobotics.ftclib.command.button.GamepadButton;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import java.util.function.Supplier;

public class ModeChangingButton {
    public enum GamepadMode {
        PRIMARY_MODE,
        SECONDARY_MODE
    }


    /**

     */
    public ModeChangingButton(GamepadEx gamepad, GamepadKeys.Button buttonKey, Command primaryCommand, Command secondaryCommand, Supplier<GamepadMode> currentMode) {
        Trigger t1 = (new GamepadButton(gamepad, buttonKey).and(new Trigger(() -> currentMode.get() == GamepadMode.PRIMARY_MODE)).whenActive(primaryCommand));
        Trigger t2 = (new GamepadButton(gamepad, buttonKey).and(new Trigger(() -> currentMode.get() == GamepadMode.SECONDARY_MODE)).whenActive(secondaryCommand));
    }


}
