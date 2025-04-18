package org.firstinspires.ftc.teamcode.TA2D2.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.TA2D2.Encoders.DriveEncoder;
import org.firstinspires.ftc.teamcode.TA2D2.PIDFController;
import org.firstinspires.ftc.teamcode.TA2D2.Poses.Pose2d;
import org.firstinspires.ftc.teamcode.TA2D2.DebugUtils;
import org.firstinspires.ftc.teamcode.TA2D2.MathUtils.MathUtil;

public class MecanumDriveNew {

    // Motors
    DcMotor frontLeft, frontRight, backLeft, backRight;

    // Encoders
    DriveEncoder par1, par2, perp;

    // IMU
    private IMU imu;

    // OpMode and debug
    private LinearOpMode opMode;
    private final boolean IS_DEBUG_MODE;

    // PID Controllers
    private PIDFController xPid = new PIDFController(0, 0, 0, 0);
    private PIDFController yPid = new PIDFController(0, 0, 0, 0);
    private PIDFController zPid = new PIDFController(0, 0, 0, 0);

    // Position tracking
    private Pose2d rotatedPos = new Pose2d(0, 0, 0);
    private Pose2d unRotatedPos = new Pose2d(0, 0, 0);
    private Pose2d prevUnRotatedPos = new Pose2d(0, 0, 0);
    private Pose2d startPos = new Pose2d(0, 0, 0);

    // Tolerances
    private final double X_TOLERANCE = 0;
    private final double Y_TOLERANCE = 0;
    private final double Z_TOLERANCE = 0;

    // Encoder constants
    private final double TICKS_PER_REV = 0;
    private final double SPOOL_DIAMETER = 0;

    private final double [] INTEGRAL_BOUNDS_X = {-0, +0};
    private final double [] INTEGRAL_BOUNDS_Y = {-0, +0};
    private final double [] INTEGRAL_BOUNDS_Z = {-0, +0};

    private static final String SUBSYSTEM_NAME = "Drive";

    private int gear = 0;

    private double [] gears = {0, 0.1 , 0.5 , 1};

    // Constructor
    public MecanumDriveNew(LinearOpMode opMode, boolean isDebug) {
        this.opMode = opMode;
        this.IS_DEBUG_MODE = isDebug;

        // Init motors
        frontLeft = opMode.hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = opMode.hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = opMode.hardwareMap.get(DcMotor.class, "backLeft");
        backRight = opMode.hardwareMap.get(DcMotor.class, "backRight");

        // Init encoders
        par1 = new DriveEncoder(frontLeft, DcMotorSimple.Direction.FORWARD, TICKS_PER_REV, SPOOL_DIAMETER);
        par2 = new DriveEncoder(frontRight, DcMotorSimple.Direction.FORWARD, TICKS_PER_REV, SPOOL_DIAMETER);
        perp = new DriveEncoder(backRight, DcMotorSimple.Direction.FORWARD, TICKS_PER_REV, SPOOL_DIAMETER);

        // Init IMU
        imu = opMode.hardwareMap.get(IMU.class, "imu");

        init();

        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Drive Initialized");
    }

    // Initialize robot components
    public void init() {

        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        xPid.setTolerance(X_TOLERANCE);
        yPid.setTolerance(Y_TOLERANCE);
        zPid.setTolerance(Z_TOLERANCE);

        xPid.setIntegrationBounds(INTEGRAL_BOUNDS_X[0], INTEGRAL_BOUNDS_X[1]);
        yPid.setIntegrationBounds(INTEGRAL_BOUNDS_Y[0], INTEGRAL_BOUNDS_Y[1]);
        zPid.setIntegrationBounds(INTEGRAL_BOUNDS_Z[0], INTEGRAL_BOUNDS_Z[1]);

        resetEncoders();
    }

    // Reset all encoders
    public void resetEncoders() {
        par1.resetEncoder();
        par2.resetEncoder();
        perp.resetEncoder();

        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Encoders Reset");
    }

    // Update robot position
    public void update() {
        unRotatedPos.setAngle(getAngle());

        double deltaX = getX() - prevUnRotatedPos.getX();
        double deltaY = getY() - prevUnRotatedPos.getY();
        double deltaAngle = getAngle() - prevUnRotatedPos.getAngle();

        unRotatedPos.setX(unRotatedPos.getX() + deltaX);
        unRotatedPos.setY(unRotatedPos.getY() + deltaY);
        unRotatedPos.setAngle(unRotatedPos.getAngle() + deltaAngle);

        rotatedPos = unRotatedPos.copy();
        rotatedPos.rotateByDegrees(deltaAngle);

        prevUnRotatedPos = unRotatedPos.copy();

        DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, new Object[][]{
                {"deltaX", deltaX},
                {"deltaY", deltaY},
                {"deltaAngle", deltaAngle},
                {"posX", rotatedPos.getX()},
                {"posY", rotatedPos.getY()},
                {"angle", rotatedPos.getAngle()}
        });
    }

    // Get heading from IMU
    public double getAngle() {
        return MathUtil.normalizeAngleTo180(imu.getRobotYawPitchRollAngles().getYaw());
    }

    // Get X from perpendicular encoder
    public double getX() {
        return perp.getPosition();
    }

    // Get Y from parallel encoders
    public double getY() {
        return (par1.getPosition() + par2.getPosition()) / 2;
    }

    // Apply motor power
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

        DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, new Object[][]{
                {"inputX", x},
                {"inputY", y},
                {"inputZ", z},
                {"FL", leftFrontPower},
                {"FR", rightFrontPower},
                {"BL", leftBackPower},
                {"BR", rightBackPower}
        });
    }



    public void setPowerByGear(Pose2d power) {
        power.setX(getX() * gears[gear]);
        power.setY(getY() * gears[gear]);
        power.setAngle(getAngle() * gears[gear]);
        setPower(power);
    }

    // Stop all motors
    public void stop() {
        setPower(new Pose2d(0, 0, 0));
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Motors Stopped");
    }

    // Field-centric drive
    public void fieldDrive(Pose2d power) {
        power.setX(Math.pow(MathUtil.applyDeadzone(power.getX(), 0.1), 2));
        power.setY(Math.pow(MathUtil.applyDeadzone(power.getY(), 0.1), 2));
        power.setAngle(Math.pow(MathUtil.applyDeadzone(power.getAngle(), 0.1), 2));


        power.rotateByDegrees(-getAngle());
        setPower(power);
        DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Field Drive Power", power);
    }

    public void fieldGearDrive(Pose2d power) {
        power.rotateByDegrees(-getAngle());
        setPowerByGear(power);
        DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Field Drive Power", power);
    }

    // Move to position with PID
    public void pidDrive(Pose2d pose, double timeOut) {
        pose.rotateByDegrees(getAngle() - pose.getAngle());

        xPid.setTimeout(timeOut);
        yPid.setTimeout(timeOut);
        zPid.setTimeout(timeOut);

        xPid.reset();
        yPid.reset();
        zPid.reset();

        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Starting PID Drive");

        do {
            update();

            double xPower = xPid.calculate(unRotatedPos.getX(), pose.getX());
            double yPower = yPid.calculate(unRotatedPos.getY(), pose.getY());
            double zPower = zPid.calculate(unRotatedPos.getAngle(), pose.getAngle());

            setPower(new Pose2d(yPower, xPower, zPower));

            DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, new Object[][]{
                    {"xPower", xPower},
                    {"yPower", yPower},
                    {"zPower", zPower},
                    {"targetX", pose.getX()},
                    {"targetY", pose.getY()},
                    {"targetAngle", pose.getAngle()}
            });

            opMode.telemetry.update();

        } while ((!xPid.atSetPoint() || !yPid.atSetPoint() || !zPid.atSetPoint()) && opMode.opModeIsActive());

        stop();
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "PID Drive Complete");
    }

    public Pose2d getPose() {
        return rotatedPos.copy();
    }

    public Pose2d getUnRotatedPose() {
        return unRotatedPos.copy();
    }

    public int getGear() {
        return gear;
    }

    public void setGear(int gear) {
        gear = (int) MathUtil.clamp(gear, 0, gears.length-1);
        this.gear = gear;
    }

    public void setStartPos(Pose2d startPos) {
        this.startPos = startPos;
    }
}
