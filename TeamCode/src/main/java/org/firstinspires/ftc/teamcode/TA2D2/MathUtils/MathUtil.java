package org.firstinspires.ftc.teamcode.TA2D2.MathUtils;

public class MathUtil {

    /**
     * Applies a deadzone to the input value. If the input is within the deadzone, it returns 0.
     * @param input The input value to apply the deadzone to.
     * @param deadzone The threshold below which the input will be considered 0.
     * @return The input value, or 0 if it's within the deadzone.
     */
    public static double applyDeadzone(double input, double deadzone) {
        return Math.abs(input) < deadzone ? 0 : input;
    }

    /**
     * Converts encoder ticks to distance based on counts per revolution (CPR) and diameter.
     * @param CPR The counts per revolution of the encoder.
     * @param diameter The diameter of the wheel.
     * @param ticks The number of encoder ticks to convert.
     * @return The equivalent distance.
     */
    public static double convertTicksToDistance(double CPR, double diameter, double ticks) {
        return Math.PI * diameter * (ticks / CPR);
    }

    /**
     * Converts a distance to encoder ticks based on counts per revolution (CPR) and diameter.
     * @param CPR The counts per revolution of the encoder.
     * @param diameter The diameter of the wheel.
     * @param distance The distance to convert.
     * @return The equivalent encoder ticks.
     */
    public static double convertDistanceToTicks(double CPR, double diameter, double distance) {
        return (distance / (Math.PI * diameter)) * CPR;
    }

    /**
     * Converts encoder ticks to degrees based on counts per revolution (CPR).
     * @param CPR The counts per revolution of the encoder.
     * @param ticks The number of encoder ticks to convert.
     * @return The equivalent angle in degrees.
     */
    public static double convertTicksToDegrees(double CPR, double ticks) {
        return (ticks / CPR) * 360 % 360;
    }

    /**
     * Converts degrees to encoder ticks based on counts per revolution (CPR).
     * @param CPR The counts per revolution of the encoder.
     * @param degrees The angle in degrees to convert.
     * @return The equivalent encoder ticks.
     */
    public static double convertDegreesToTicks(double CPR, double degrees) {
        return (degrees / 360) * CPR;
    }

    /**
     * Converts a voltage reading to degrees (assumes 3.3V corresponds to 360 degrees).
     * @param voltage The voltage value to convert.
     * @return The equivalent angle in degrees.
     */
    public static double voltageToDegrees(double voltage) {
        return (voltage * 360) / 3.3;
    }

    /**
     * Checks if a value is within a specified tolerance of a desired value.
     * @param desiredPosition The desired position.
     * @param currentPosition The current position.
     * @param tolerance The tolerance within which the position is considered acceptable.
     * @return True if the current position is within the tolerance of the desired position.
     */
    public static boolean inTolerance(double desiredPosition, double currentPosition, double tolerance) {
        return Math.abs(desiredPosition - currentPosition) <= tolerance;
    }

    /**
     * Optimizes an angle to minimize rotation (keeps it close to the previous angle).
     * @param angle The new angle.
     * @param prevAngle The previous angle.
     * @return The optimized angle.
     */
    public static double optimizeAngle(double angle, double prevAngle) {
        double delta = angle - prevAngle;
        if (Math.abs(delta) > 180.0) {
            angle -= Math.signum(delta) * 360.0;
        }
        return angle;
    }

    /**
     * Clamps a value between a minimum and maximum range.
     * @param value The value to clamp.
     * @param min The minimum allowable value.
     * @param max The maximum allowable value.
     * @return The clamped value.
     */
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Linearly interpolates between two values based on a given fraction.
     * @param start The starting value.
     * @param end The ending value.
     * @param fraction The fraction to interpolate by.
     * @return The interpolated value.
     */
    public static double lerp(double start, double end, double fraction) {
        return start + fraction * (end - start);
    }

    /**
     * Maps a value from one range to another.
     * @param value The value to map.
     * @param inMin The minimum value of the input range.
     * @param inMax The maximum value of the input range.
     * @param outMin The minimum value of the output range.
     * @param outMax The maximum value of the output range.
     * @return The mapped value.
     */
    public static double map(double value, double inMin, double inMax, double outMin, double outMax) {
        return outMin + ((value - inMin) * (outMax - outMin)) / (inMax - inMin);
    }

    /**
     * Normalizes an angle to the range [0, 360).
     * @param angle The angle to normalize.
     * @return The normalized angle in the range [0, 360).
     */
    public static double normalizeAngle(double angle) {
        angle %= 360;
        return angle < 0 ? angle + 360 : angle;
    }

    /**
     * Normalizes an angle to the range [-180, 180).
     * @param angle The angle to normalize.
     * @return The normalized angle in the range [-180, 180).
     */
    public static double normalizeAngleTo180(double angle) {
        angle = normalizeAngle(angle);
        return angle > 180 ? angle - 360 : angle;
    }

    /**
     * Calculates the distance between two points.
     * @param x1 The x-coordinate of the first point.
     * @param y1 The y-coordinate of the first point.
     * @param x2 The x-coordinate of the second point.
     * @param y2 The y-coordinate of the second point.
     * @return The distance between the two points.
     */
    public static double distance(double x1, double y1, double x2, double y2) {
        double dx = x2 - x1;
        double dy = y2 - y1;
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Calculates the angle between two points in degrees.
     * @param x1 The x-coordinate of the first point.
     * @param y1 The y-coordinate of the first point.
     * @param x2 The x-coordinate of the second point.
     * @param y2 The y-coordinate of the second point.
     * @return The angle between the two points in degrees.
     */
    public static double angleBetweenPoints(double x1, double y1, double x2, double y2) {
        return Math.toDegrees(Math.atan2(y2 - y1, x2 - x1));
    }

    /**
     * Wraps a value to a specific range [min, max).
     * @param value The value to wrap.
     * @param min The minimum value of the range.
     * @param max The maximum value of the range.
     * @return The wrapped value.
     */
    public static double wrap(double value, double min, double max) {
        double range = max - min;
        value = (value - min) % range;
        return value < 0 ? value + max : value + min;
    }
}