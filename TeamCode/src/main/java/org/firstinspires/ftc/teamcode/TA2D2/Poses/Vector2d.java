package org.firstinspires.ftc.teamcode.TA2D2.Poses;

/**
 * Represents a 2D vector with x and y coordinates. Provides utility methods for common vector operations.
 * This version is optimized for performance, readability, and maintainability.
 */
public class Vector2d {
    private double x;
    private double y;

    /**
     * Constructs a new Vector2d with the specified x and y coordinates.
     *
     * @param x the x-coordinate of the vector
     * @param y the y-coordinate of the vector
     */
    public Vector2d(double x, double y){
        this.x = x;
        this.y = y;
    }

    /**
     * Constructs a new Vector2d from an existing Vector2d.
     *
     * @param vector2D the vector to copy from
     */
    public Vector2d(Vector2d vector2D) {
        this(vector2D.x, vector2D.y);
    }

    /**
     * Rotates the vector by a specified angle.
     * This method does not modify the current vector but returns a new one.
     *
     * @param angle the angle in radians by which to rotate the vector
     * @return a new Vector2d instance representing the rotated vector
     */
    public Vector2d rotateBy(double angle) {
        // Precompute sine and cosine to avoid multiple calls to Math functions
        double cos = Math.cos(angle);
        double sin = Math.sin(angle);
        // Efficiently calculate rotated coordinates
        double newX = this.x * cos - this.y * sin;
        double newY = this.x * sin + this.y * cos;
        return new Vector2d(newX, newY);
    }

    /**
     * Rotates the vector by a specified angle in degrees.
     * Converts degrees to radians and calls the rotateBy method.
     *
     * @param degrees the angle in degrees by which to rotate the vector
     * @return a new Vector2d instance representing the rotated vector
     */
    public Vector2d rotateByDegrees(double degrees) {
        return rotateBy(Math.toRadians(degrees));
    }



    /**
     * Returns the magnitude (length) of the vector.
     *
     * @return the magnitude of the vector
     */
    public double magnitude() {
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Normalizes the vector, scaling it to have a magnitude of 1.
     * Returns a new vector, leaving the original unchanged.
     * If the vector has zero magnitude, it returns a zero vector.
     *
     * @return a new Vector2d instance representing the normalized vector
     */
    public Vector2d normalize() {
        double magnitude = magnitude();
        // Avoid division by zero in case of a zero vector
        if (magnitude == 0) {
            return new Vector2d(0, 0);
        }
        return new Vector2d(x / magnitude, y / magnitude);
    }

    /**
     * Adds another vector to this vector and returns a new vector.
     *
     * @param vector the vector to add
     * @return a new Vector2d instance representing the sum of the two vectors
     */
    public Vector2d add(Vector2d vector) {
        return new Vector2d(this.x + vector.x, this.y + vector.y);
    }

    /**
     * Subtracts another vector from this vector and returns a new vector.
     *
     * @param vector the vector to subtract
     * @return a new Vector2d instance representing the difference between the two vectors
     */
    public Vector2d subtract(Vector2d vector) {
        return new Vector2d(this.x - vector.x, this.y - vector.y);
    }

    /**
     * Scales the vector by a specified factor and returns a new vector.
     *
     * @param scalar the factor by which to scale the vector
     * @return a new Vector2d instance representing the scaled vector
     */
    public Vector2d scale(double scalar) {
        return new Vector2d(this.x * scalar, this.y * scalar);
    }

    /**
     * Returns the angle of the vector relative to the positive x-axis.
     * The angle is in radians.
     *
     * @return the angle of the vector in radians
     */
    public double angle() {
        return Math.atan2(y, x);
    }

    /**
     * Gets the x-coordinate of the vector.
     *
     * @return the x-coordinate of the vector
     */
    public double getX() {
        return x;
    }

    /**
     * Sets the x-coordinate of the vector.
     *
     * @param x the new x-coordinate value
     */
    public void setX(double x) {
        this.x = x;
    }

    /**
     * Gets the y-coordinate of the vector.
     *
     * @return the y-coordinate of the vector
     */
    public double getY() {
        return y;
    }

    /**
     * Sets the y-coordinate of the vector.
     *
     * @param y the new y-coordinate value
     */
    public void setY(double y) {
        this.y = y;
    }
}
