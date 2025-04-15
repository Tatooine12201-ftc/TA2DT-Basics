package org.firstinspires.ftc.teamcode.TA2D2.Pathing;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.TA2D2.Poses.Pose2d;
import org.firstinspires.ftc.teamcode.TA2D2.SubSystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.TA2D2.MathUtils.MathUtil;

/**
 * A class to follow paths using splines, with support for strafing, slant (diagonal) movement, and motion profiling.
 */
@Disabled
public class PathFollower {
    private final MecanumDrive drive;
    private final Spline xSpline, ySpline;
    private double t = 0;
    private boolean isStrafing = false;
    private boolean isSlanting = false;
    private double maxSpeed = 1.0;
    private double acceleration = 0.02; // Adjust acceleration for smoother motion
    private double currentSpeed = 0.0;

    public PathFollower(MecanumDrive drive, Pose2d start, Pose2d end) {
        this.drive = drive;
        this.xSpline = new Spline(start.getX(), end.getX(), start.getAngle(), end.getAngle());
        this.ySpline = new Spline(start.getY(), end.getY(), start.getAngle(), end.getAngle());
    }

    public void setStrafing(boolean isStrafing) {
        this.isStrafing = isStrafing;
    }

    public void setSlanting(boolean isSlanting) {
        this.isSlanting = isSlanting;
    }

    public void setMaxSpeed(double speed) {
        this.maxSpeed = MathUtil.clamp(speed, 0, 1);
    }

    public void setAcceleration(double acceleration) {
        this.acceleration = Math.max(0.01, acceleration); // Prevent zero acceleration
    }

    public void followPath(double deltaT) {
        if (t >= 1.0) {
            drive.stop();
            return;
        }

        currentSpeed = Math.min(currentSpeed + acceleration, maxSpeed);

        double targetX = xSpline.evaluate(t);
        double targetY = ySpline.evaluate(t);
        double targetAngle = calculateTargetAngle(t);
        Pose2d targetPose = new Pose2d(targetX, targetY, Math.toDegrees(targetAngle));

        if (isStrafing) {
            targetPose = new Pose2d(targetPose.getX(), targetPose.getY(), drive.getAngle());
        }

        if (isSlanting) {
            double forwardPower = targetPose.getY();
            double strafePower = targetPose.getX();
            targetPose = new Pose2d(strafePower, forwardPower, targetPose.getAngle());
        }

        targetPose = new Pose2d(
                targetPose.getX() * currentSpeed,
                targetPose.getY() * currentSpeed,
                targetPose.getAngle() * currentSpeed
        );

        drive.pidDrive(targetPose, 0.1);
        t += deltaT * currentSpeed;
    }

    private double calculateTargetAngle(double t) {
        double deltaX = xSpline.evaluateDerivative(t);
        double deltaY = ySpline.evaluateDerivative(t);
        return Math.atan2(deltaY, deltaX);
    }

    public void reset() {
        t = 0;
        currentSpeed = 0;
    }

    public boolean isPathComplete() {
        return t >= 1.0;
    }
}