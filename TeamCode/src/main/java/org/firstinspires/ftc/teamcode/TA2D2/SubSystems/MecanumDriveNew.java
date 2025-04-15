package org.firstinspires.ftc.teamcode.TA2D2.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.TA2D2.DriveEncoder;
import org.firstinspires.ftc.teamcode.TA2D2.PIDFController;
import org.firstinspires.ftc.teamcode.TA2D2.Poses.Pose2d;

public class MecanumDriveNew {

    DcMotor frontLeft, frontRight, backLeft, backRight;

    DriveEncoder par1, par2, perp;

    private IMU imu;

    private LinearOpMode opMode;
    private final boolean IS_DEBUG_MODE;

    private PIDFController xPid = new PIDFController(0, 0, 0, 0);
    private PIDFController yPid = new PIDFController(0, 0, 0, 0);
    private PIDFController zPid = new PIDFController(0, 0, 0, 0);

    private Pose2d rotatedPos = new Pose2d(0, 0, 0);
    private Pose2d unRotatedPos = new Pose2d(0, 0, 0);
    private Pose2d prevUnRotatedPos = new Pose2d(0, 0, 0);

    private final double xTolerance = 0;
    private final double yTolerance = 0;
    private final double zTolerance = 0;

    private final double TICK_PER_REV = 0;
    private final double SPOOL_DIAMETER = 0;

    private static final String SUBSYSTEM_NAME = "Drive";

    public MecanumDriveNew(LinearOpMode opMode, boolean isDebug) {
        this.opMode = opMode;
        this.IS_DEBUG_MODE = isDebug;

        frontLeft = opMode.hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = opMode.hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = opMode.hardwareMap.get(DcMotor.class, "backLeft");
        backRight = opMode.hardwareMap.get(DcMotor.class, "backRight");

        par1 = new DriveEncoder(opMode.hardwareMap.get(DcMotor.class, "frontLeft"), DcMotorSimple.Direction.FORWARD, TICK_PER_REV, SPOOL_DIAMETER);
        par2 = new DriveEncoder(opMode.hardwareMap.get(DcMotor.class, "frontRight"), DcMotorSimple.Direction.FORWARD, TICK_PER_REV, SPOOL_DIAMETER);
        perp = new DriveEncoder(opMode.hardwareMap.get(DcMotor.class, "backRight"), DcMotorSimple.Direction.FORWARD, TICK_PER_REV, SPOOL_DIAMETER);

        imu = opMode.hardwareMap.get(IMU.class, "imu");

        init();

    }

    public void init() {
        // Set motors to BRAKE mode when power is zero (stopping the robot)
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set PID tolerances
        xPid.setTolerance(xTolerance);
        yPid.setTolerance(yTolerance);
        zPid.setTolerance(zTolerance);

        resetEncoders();
    }

    public void resetEncoders() {
        par1.resetEncoder();
        par2.resetEncoder();
        perp.resetEncoder();
    }

    public void update() {
        unRotatedPos.setAngle(getAngle());

        double deltaX = getX() - prevUnRotatedPos.getX();

        double deltaY = getY() - prevUnRotatedPos.getY();

        double deltaAngle = getAngle() - prevUnRotatedPos.getAngle();

        unRotatedPos.setX(getX() + deltaX);
        unRotatedPos.setY(getY() + deltaY);
        unRotatedPos.setAngle(getAngle() + deltaAngle);

        rotatedPos  = unRotatedPos;

        rotatedPos.rotateByDegrees(deltaAngle);

        prevUnRotatedPos = unRotatedPos;

    }

    public double getAngle() {
        return imu.getRobotYawPitchRollAngles().getYaw();
    }

    public double getX(){
        return perp.getPosition();
    }

    public double getY() {
        return (par1.getPosition() + par2.getPosition()) / 2;
    }

    public void setPower(Pose2d power) {
        double x = power.getX();
        double y = power.getY();
        double z = power.getAngle();

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(z), 1);

        double rightFrontPower = (y - x - z) / denominator;
        double rightBackPower = (y + x - z) / denominator;
        double leftFrontPower = (y + x + z) / denominator;
        double leftBackPower = (y - x + z) / denominator;

        frontLeft.setPower(leftFrontPower);
        backLeft.setPower(leftBackPower);
        frontRight.setPower(rightFrontPower);
        backRight.setPower(rightBackPower);
    }

    public void stop() {
        setPower(new Pose2d(0, 0, 0));
    }

    public void fieldDrive(Pose2d power){
        power.rotateByDegrees(getAngle());
        setPower(power);
    }

    public void pidDrive(Pose2d pose, double timeOut){
        pose.unrotateByDegrees(getAngle());

        xPid.setTimeout(timeOut);
        yPid.setTimeout(timeOut);
        zPid.setTimeout(timeOut);

        xPid.reset();
        yPid.reset();
        zPid.reset();

        do {
            update();
            double xPower = xPid.calculate(unRotatedPos.getX(), pose.getX());
            double yPower = yPid.calculate(unRotatedPos.getY(), pose.getY());
            double zPower = zPid.calculate(unRotatedPos.getAngle(), pose.getAngle());

            setPower(new Pose2d(yPower, xPower, zPower));
        }
        while ((!xPid.atSetPoint() || !yPid.atSetPoint() || !zPid.atSetPoint()) && opMode.opModeIsActive());
        stop();
    }
}
