package org.firstinspires.ftc.teamcode.TA2D2.Gamepads;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.TA2D2.DebugUtils;
import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * A utility class that simplifies checking button presses, stick movements,
 * and triggers for a single gamepad. It also tracks the previous state of the
 * gamepad to detect changes or newly pressed buttons.
 *
 * Usage:
 * <pre>
 *     GamepadEx egp = new GamepadEx(gamepad1);
 *     egp.update(gamepad1);
 *     if (egp.justPressedButton(GamepadKeys.Button.CIRCLE)) {
 *         // Circle was pressed this frame, but not last frame
 *     }
 *     double leftY = egp.getStick(GamepadKeys.Stick.LEFT_STICK_Y);
 * </pre>
 */
public class GamepadEx {
    private Gamepad gamepad;
    private final Gamepad prevGamepad;
    private long lastPressTime = 0;

    /**
     * Creates a new GamepadEx instance with an initial gamepad reference.
     *
     * @param gamepad The initial gamepad reference (e.g., gamepad1).
     */
    public GamepadEx(Gamepad gamepad) {
        this.gamepad = gamepad;
        this.prevGamepad = new Gamepad();
    }

    /**
     * Updates the GamepadEx state each loop to track changes.
     * Call this method at the start of your loop.
     *
     * @param gamepad The updated gamepad reference.
     */
    public void update(Gamepad gamepad) {
        prevGamepad.copy(this.gamepad);
        this.gamepad.copy(gamepad);
    }

    public boolean getButton(GamepadKeys.Button button) {
        return getButtonState(gamepad, button);
    }

    public boolean justPressedButton(GamepadKeys.Button button) {
        return getButtonState(gamepad, button) && !getButtonState(prevGamepad, button);
    }

    private boolean getButtonState(Gamepad gp, GamepadKeys.Button button) {
        switch (button) {
            case CROSS:
                return gp.cross;
            case CIRCLE:
                return gp.circle;
            case TRIANGLE:
                return gp.triangle;
            case SQUARE:
                return gp.square;
            case LEFT_BUMPER:
                return gp.left_bumper;
            case RIGHT_BUMPER:
                return gp.right_bumper;
            case DPAD_UP:
                return gp.dpad_up;
            case DPAD_DOWN:
                return gp.dpad_down;
            case DPAD_LEFT:
                return gp.dpad_left;
            case DPAD_RIGHT:
                return gp.dpad_right;
            case BACK:
                return gp.back;
            case START:
                return gp.start;
            case LEFT_STICK_BUTTON:
                return gp.left_stick_button;
            case RIGHT_STICK_BUTTON:
                return gp.right_stick_button;
            default:
                return false;
        }
    }

    public double getStick(GamepadKeys.Stick stick) {
        switch (stick) {
            case LEFT_STICK_X:
                return gamepad.left_stick_x;
            case LEFT_STICK_Y:
                return -gamepad.left_stick_y;
            case RIGHT_STICK_X:
                return gamepad.right_stick_x;
            case RIGHT_STICK_Y:
                return -gamepad.right_stick_y;
            default:
                return 0;
        }
    }

    public boolean stateChangedStick(GamepadKeys.Stick stick) {
        switch (stick) {
            case LEFT_STICK_X:
                return gamepad.left_stick_x != prevGamepad.left_stick_x;
            case LEFT_STICK_Y:
                return gamepad.left_stick_y != prevGamepad.left_stick_y;
            case RIGHT_STICK_X:
                return gamepad.right_stick_x != prevGamepad.right_stick_x;
            case RIGHT_STICK_Y:
                return gamepad.right_stick_y != prevGamepad.right_stick_y;
            default:
                return false;
        }
    }

    public double getTrigger(GamepadKeys.Trigger trigger) {
        return trigger == GamepadKeys.Trigger.LEFT_TRIGGER ? gamepad.left_trigger : gamepad.right_trigger;
    }

    public boolean justPressedTrigger(GamepadKeys.Trigger trigger, double threshold) {
        double currentValue = getTrigger(trigger);
        double previousValue = trigger == GamepadKeys.Trigger.LEFT_TRIGGER ? prevGamepad.left_trigger : prevGamepad.right_trigger;
        return currentValue > threshold && previousValue <= threshold;
    }

    public long getButtonHoldDuration(GamepadKeys.Button button) {
        if (getButton(button)) {
            if (lastPressTime == 0) lastPressTime = System.currentTimeMillis();
            return System.currentTimeMillis() - lastPressTime;
        }
        lastPressTime = 0;
        return 0;
    }

    public void debugGamepadState(Telemetry telemetry, boolean debugMode) {
        DebugUtils.logDebug(telemetry, debugMode, "GamepadEx", "Left Stick X", gamepad.left_stick_x);
        DebugUtils.logDebug(telemetry, debugMode, "GamepadEx", "Left Stick Y", gamepad.left_stick_y);
        DebugUtils.logDebug(telemetry, debugMode, "GamepadEx", "Right Stick X", gamepad.right_stick_x);
        DebugUtils.logDebug(telemetry, debugMode, "GamepadEx", "Right Stick Y", gamepad.right_stick_y);
        DebugUtils.logDebug(telemetry, debugMode, "GamepadEx", "Left Trigger", gamepad.left_trigger);
        DebugUtils.logDebug(telemetry, debugMode, "GamepadEx", "Right Trigger", gamepad.right_trigger);
    }
}
