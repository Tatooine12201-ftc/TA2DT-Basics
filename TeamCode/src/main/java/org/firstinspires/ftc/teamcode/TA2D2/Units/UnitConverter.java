package org.firstinspires.ftc.teamcode.TA2D2.Units;

/**
 * Utility class for converting between different units of length.
 * Supports conversions between inches, centimeters, millimeters, feet, and meters.
 */
public class UnitConverter {

    /**
     * Converts a value from one unit to another.
     *
     * @param value The value to convert.
     * @param from The unit to convert from.
     * @param to The unit to convert to.
     * @return The converted value.
     */
    public static double convert(double value, unit from, unit to) {
        // If the 'from' and 'to' units are the same, no conversion is needed
        if (from == to) {
            return value;
        }

        // Perform conversions based on the 'from' unit
        switch (from) {
            case INCHES:
                // Converting from INCHES to other units
                switch (to) {
                    case CM:  // Inches to Centimeters
                        return value * 2.54;
                    case MM:  // Inches to Millimeters
                        return value * 25.4;
                    case FEET:  // Inches to Feet
                        return value / 12;
                    case METERS:  // Inches to Meters
                        return value * 0.0254;
                }
                break;
            case CM:
                // Converting from CM to other units
                switch (to) {
                    case INCHES:  // Centimeters to Inches
                        return value / 2.54;
                    case MM:  // Centimeters to Millimeters
                        return value * 10;
                    case FEET:  // Centimeters to Feet
                        return value / 30.48;
                    case METERS:  // Centimeters to Meters
                        return value * 0.01;
                }
                break;
            case MM:
                // Converting from MM to other units
                switch (to) {
                    case INCHES:  // Millimeters to Inches
                        return value / 25.4;
                    case CM:  // Millimeters to Centimeters
                        return value / 10;
                    case FEET:  // Millimeters to Feet
                        return value / 304.8;
                    case METERS:  // Millimeters to Meters
                        return value * 0.001;
                }
                break;
            case FEET:
                // Converting from FEET to other units
                switch (to) {
                    case INCHES:  // Feet to Inches
                        return value * 12;
                    case CM:  // Feet to Centimeters
                        return value * 30.48;
                    case MM:  // Feet to Millimeters
                        return value * 304.8;
                    case METERS:  // Feet to Meters
                        return value * 0.3048;
                }
                break;
            case METERS:
                // Converting from METERS to other units
                switch (to) {
                    case INCHES:  // Meters to Inches
                        return value / 0.0254;
                    case CM:  // Meters to Centimeters
                        return value / 0.01;
                    case MM:  // Meters to Millimeters
                        return value / 0.001;
                    case FEET:  // Meters to Feet
                        return value / 0.3048;
                }
                break;
        }
        // Return 0 if the conversion could not be completed (e.g., invalid units)
        return 0;
    }
}