package org.firstinspires.ftc.teamcode.TA2D2.Poses;

public class Pose2d extends Vector2d{

    private double angle = 0;

    public Pose2d(double x, double y , double angle) {
        super(x, y);
        setAngle(angle);

    }

    public Pose2d(Vector2d vector2D, double angle) {
        this(vector2D.getX(), vector2D.getY(), angle);
    }

    public Pose2d copy() {
        return new Pose2d(getX(), getY(), getAngle());
    }

    public double getAngle() {
        return angle;
    }

    public void setAngle(double angle) {
        this.angle = angle;
    }


}
