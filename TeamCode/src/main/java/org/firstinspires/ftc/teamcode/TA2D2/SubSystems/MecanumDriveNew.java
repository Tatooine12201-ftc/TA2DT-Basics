package org.firstinspires.ftc.teamcode.TA2D2.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.TA2D2.Encoders.DriveEncoder;
import org.firstinspires.ftc.teamcode.TA2D2.PID.PIDController;
import org.firstinspires.ftc.teamcode.TA2D2.Poses.Pose2d;
import org.firstinspires.ftc.teamcode.TA2D2.DebugUtils;
import org.firstinspires.ftc.teamcode.TA2D2.MathUtils.MathUtil;

public class MecanumDriveNew {

    // --- Constants ---
    private static final String SUBSYSTEM_NAME = "Drive";
    private final double X_TOLERANCE = 0;
    private final double Y_TOLERANCE = 0;
    private final double Z_TOLERANCE = 0;
    private final double TICKS_PER_REV = 8192;
    private final double SPOOL_DIAMETER = 3.5;

    private final double LATERAL_DISTANCE = 21;

    private final double FORWARD_OFFSET = -15;
    private final double[] INTEGRAL_BOUNDS_X = {-0, +0};
    private final double[] INTEGRAL_BOUNDS_Y = {-0, +0};
    private final double[] INTEGRAL_BOUNDS_Z = {-0, +0};
    private final double[] gears = {0, 0.1, 0.5, 1};

    // --- Hardware ---
    private DcMotor frontLeft, frontRight, backLeft, backRight;
    private DriveEncoder par1, par2, perp;
    private IMU imu;

    // --- OpMode & Config ---
    private LinearOpMode opMode;
    private final boolean IS_DEBUG_MODE;

    // --- PID Controllers ---
    private PIDController xPid = new PIDController(0, 0, 0, 0);
    private PIDController yPid = new PIDController(0, 0, 0, 0);
    private PIDController zPid = new PIDController(0, 0, 0, 0);

    // --- Position tracking ---
    private Pose2d fieldPos = new Pose2d(0, 0, 0);
    private Pose2d robotPos = new Pose2d(0, 0, 0);
    private Pose2d deltaFieldPos = new Pose2d(0, 0, 0);
    private Pose2d prevRawPos = new Pose2d(0, 0, 0);
    private Pose2d startPos = new Pose2d(0, 0, 0);

    // --- Gear setting ---
    private int gear = 0;

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
        par1 = new DriveEncoder(backRight, DcMotorSimple.Direction.REVERSE, TICKS_PER_REV, SPOOL_DIAMETER);//right
        par2 = new DriveEncoder(frontLeft, DcMotorSimple.Direction.FORWARD, TICKS_PER_REV, SPOOL_DIAMETER);//left
        perp = new DriveEncoder(backLeft, DcMotorSimple.Direction.FORWARD, TICKS_PER_REV, SPOOL_DIAMETER);

        // Init IMU
        imu = opMode.hardwareMap.get(IMU.class, "imu");

        init();

        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Drive Initialized");
    }

    // Initialize motors, PID, directions
    public void init() {
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        imu.resetYaw();

        xPid.setTolerance(X_TOLERANCE);
        yPid.setTolerance(Y_TOLERANCE);
        zPid.setTolerance(Z_TOLERANCE);

        xPid.setIntegrationBounds(INTEGRAL_BOUNDS_X[0], INTEGRAL_BOUNDS_X[1]);
        yPid.setIntegrationBounds(INTEGRAL_BOUNDS_Y[0], INTEGRAL_BOUNDS_Y[1]);
        zPid.setIntegrationBounds(INTEGRAL_BOUNDS_Z[0], INTEGRAL_BOUNDS_Z[1]);

        resetEncoders();
    }

    // Reset encoders to zero
    public void resetEncoders() {
        par1.resetEncoder();
        par2.resetEncoder();
        perp.resetEncoder();
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Encoders Reset");
    }

    // Update robot's position with odometry and IMU
    public void update() {
        double x = getRawX();
        double y = getRawY();
        double angle = getAngle();
        robotPos.setAngle(angle);
        fieldPos.setAngle(angle);

        deltaFieldPos.setX(getRawX() - prevRawPos.getX());
        deltaFieldPos.setY(getRawY() - prevRawPos.getY());
        deltaFieldPos.setAngle(angle - prevRawPos.getAngle());

        double phi = (getRawY1() - getRawY2())/LATERAL_DISTANCE;

        deltaFieldPos.setX(deltaFieldPos.getX() - FORWARD_OFFSET * phi);

        deltaFieldPos.rotateByDegrees(-angle);

        fieldPos.setX(fieldPos.getX() + deltaFieldPos.getX());
        fieldPos.setY(fieldPos.getY() + deltaFieldPos.getY());

        robotPos = fieldPos.copy();
        robotPos.rotateByDegrees(angle);
        
        prevRawPos.setX(x);
        prevRawPos.setY(y);
        prevRawPos.setAngle(angle);
        
        

        DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, new Object[][]{
                {"deltaRawPosX", getRawX()- prevRawPos.getX()},
                {"deltaRawPosY", getRawY()- prevRawPos.getY()},
                {"deltaX", deltaFieldPos.getX()},
                {"deltaY", deltaFieldPos.getY()},
                {"deltaAngle", deltaFieldPos.getAngle()},
                {"posX", fieldPos.getX()},
                {"posY", fieldPos.getY()},
                {"angle", fieldPos.getAngle()}
        });
    }

    // Get heading from IMU
    public double getAngle() {
        return -MathUtil.normalizeAngleTo180(imu.getRobotYawPitchRollAngles().getYaw());
    }

    // Get X from perp encoder
    public double getRawX() {
        DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "perp ticks", perp.getPosition());
        return MathUtil.convertTicksToDistance(TICKS_PER_REV, SPOOL_DIAMETER, perp.getPosition());
    }

    public double getRawY1() {
        DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "par1 ticks", par1.getPosition());
        return MathUtil.convertTicksToDistance(TICKS_PER_REV, SPOOL_DIAMETER, par1.getPosition());
    }
    public double getRawY2() {
        DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "par2 ticks", par2.getPosition());
        return MathUtil.convertTicksToDistance(TICKS_PER_REV, SPOOL_DIAMETER, par2.getPosition());
    }

    // Get Y from avg of parallel encoders
    public double getRawY() {
        DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "par2 ticks", (getRawY1() + getRawY2()) / 2);
        return (getRawY1() + getRawY2()) / 2;
    }

    // Set motor powers from Pose2d input

    public void setPowerFrontRight(double power){
        frontRight.setPower(power);
    }
    public void setPowerFrontLeft(double power){
        frontLeft.setPower(power);
    }
    public void setPowerBackRight(double power){
        backRight.setPower(power);
    }
    public void setPowerBackLeft(double power){
        backLeft.setPower(power);
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

        frontRight.setPower(rightFrontPower);
        frontLeft.setPower(leftFrontPower);
        backRight.setPower(rightBackPower);
        backLeft.setPower(leftBackPower);

        update();

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

    // Drive with gears scaling the input power
    public void setPowerByGear(Pose2d power) {
        power.setX(getRawX() * gears[gear]);
        power.setY(getRawY() * gears[gear]);
        power.setAngle(getAngle() * gears[gear]);
        setPower(power);
    }

    // Stop all movement
    public void stop() {
        setPower(new Pose2d(0, 0, 0));
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Motors Stopped");
    }

    // Field-oriented drive
    public void fieldDrive(Pose2d power) {
        power.setX(MathUtil.applyDeadzone(power.getX(), 0.1));
        power.setY(MathUtil.applyDeadzone(power.getY(), 0.1));
        power.setAngle(MathUtil.applyDeadzone(power.getAngle(), 0.1));

        power.rotateByDegrees(-getAngle());
        setPower(power);
        DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Field Drive Power", power);
    }

    // Field-oriented drive with gears
    public void fieldGearDrive(Pose2d power) {
        power.rotateByDegrees(-getAngle());
        setPowerByGear(power);
        DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Field Drive Power", power);
    }

    // Drive to pose using PID controllers
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

            double xPower = xPid.calculate(robotPos.getX(), pose.getX());
            double yPower = yPid.calculate(robotPos.getY(), pose.getY());
            double zPower = zPid.calculate(robotPos.getAngle(), pose.getAngle());

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

    // Getters and setters
    public int getGear() {
        return gear;
    }

    public void setGear(int gear) {
        gear = (int) MathUtil.clamp(gear, 0, gears.length - 1);
        this.gear = gear;
    }

    public void setStartPos(Pose2d startPos) {
        this.startPos = startPos;
    }
}