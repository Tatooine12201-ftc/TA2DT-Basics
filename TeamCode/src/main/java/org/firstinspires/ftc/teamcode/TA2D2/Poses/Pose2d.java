package org.firstinspires.ftc.teamcode.TA2D2.Poses;

/**
 * Represents a 2D pose consisting of a position (x, y) and an angle (heading).
 */
public class Pose2d {
    private Vector2d position;
    private double angle = 0;

    /**
     * Constructor to initialize a Pose2d object with the specified position (x, y) and angle.
     *
     * @param x The x-coordinate of the position.
     * @param y The y-coordinate of the position.
     * @param angle The angle (heading) in radians.
     */
    public Pose2d(double x, double y, double angle){
        this.position = new Vector2d(x, y);
        this.angle = angle;
    }

    /**
     * Constructor to initialize a Pose2d object from a Vector2d (position) and angle.
     *
     * @param vec The position as a Vector2d.
     * @param angle The angle (heading) in radians.
     */
    public Pose2d(Vector2d vec, double angle){
        this.position = vec;
        this.angle = angle;
    }

    /**
     * Copy constructor to create a Pose2d object from another Pose2d.
     *
     * @param pose The Pose2d object to copy.
     */
    public Pose2d(Pose2d pose){
        this(pose.position, pose.getAngle());  // Reuse the main constructor
    }

    /**
     * Gets the x-coordinate of the pose.
     *
     * @return The x-coordinate.
     */
    public double getX() {
        return position.getX();
    }

    /**
     * Gets the y-coordinate of the pose.
     *
     * @return The y-coordinate.
     */
    public double getY() {
        return position.getY();
    }

    /**
     * Gets the angle (heading) of the pose.
     *
     * @return The angle in radians.
     */
    public double getAngle() {
        return angle;
    }

    /**
     * Sets the x-coordinate of the pose.
     *
     * @param x The new x-coordinate.
     */
    public void setX(double x) {
        position.setX(x);
    }

    /**
     * Sets the y-coordinate of the pose.
     *
     * @param y The new y-coordinate.
     */
    public void setY(double y) {
        position.setY(y);
    }

    /**
     * Sets the angle (heading) of the pose.
     *
     * @param angle The new angle in radians.
     */
    public void setAngle(double angle) {
        this.angle = angle;
    }

    /**
     * Rotates the pose's position around the origin by the specified angle.
     *
     * @param rotationAngle The angle in radians to rotate the pose by.
     */
    public void rotate(double rotationAngle) {
        // Rotate the position (Vector2d) by the specified angle
        position = position.rotateBy(rotationAngle);


    }

    /**
     * Unrotates the pose's position around the origin by the specified angle.
     *
     * @param unrotationAngle The angle in radians to unrotate the pose by.
     */
    public void unrotate(double unrotationAngle) {
        // Unrotate the position (Vector2d) by the inverse of the specified angle
        position = position.unRotateBy(unrotationAngle);

    }

    /**
     * Rotates the pose by the specified angle in degrees.
     * Uses the rotateByDegrees method from Vector2d.
     *
     * @param rotationDegrees The angle in degrees to rotate the pose by.
     */
    public void rotateByDegrees(double rotationDegrees) {
        // Rotate the position using Vector2d's rotateByDegrees method
        position = position.rotateByDegrees(rotationDegrees);
    }

    /**
     * Unrotates the pose by the specified angle in degrees.
     * Uses the unRotateByDegrees method from Vector2d.
     *
     * @param unrotationDegrees The angle in degrees to unrotate the pose by.
     */
    public void unrotateByDegrees(double unrotationDegrees) {
        // Unrotate the position using Vector2d's unRotateByDegrees method
        position = position.unRotateByDegrees(unrotationDegrees);
    }

    /**
     * Converts the Pose2d to a string representation.
     *
     * @return A string describing the pose.
     */
    @Override
    public String toString() {
        return String.format("Pose2d(x: %.2f, y: %.2f, angle: %.2f radians)", getX(), getY(), angle);
    }

    /**
     * Normalizes the angle to the range [0, 2π).
     *
     * @param angle The angle in radians to normalize.
     * @return The normalized angle in radians.
     */
    private double normalizeAngle(double angle) {
        angle %= (2 * Math.PI);
        return angle < 0 ? angle + 2 * Math.PI : angle;
    }
}
