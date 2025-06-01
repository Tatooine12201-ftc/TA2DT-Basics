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

public class MecanumDrive {

    // --- Constants ---
    private static final String SUBSYSTEM_NAME = "Drive";  // Name of the subsystem
    private final double X_TOLERANCE = 0;  // Tolerance for the X-axis during PID control
    private final double Y_TOLERANCE = 0;  // Tolerance for the Y-axis during PID control
    private final double Z_TOLERANCE = 0;  // Tolerance for the Z-axis (rotation) during PID control
    private final double TICKS_PER_REV = 0;  // Number of encoder ticks per revolution of the wheel
    private final double SPOOL_DIAMETER = 0;  // Diameter of the spool for the encoder
    private final double[] INTEGRAL_BOUNDS_X = {-0, +0};  // Integral bounds for the X-axis PID controller
    private final double[] INTEGRAL_BOUNDS_Y = {-0, +0};  // Integral bounds for the Y-axis PID controller
    private final double[] INTEGRAL_BOUNDS_Z = {-0, +0};  // Integral bounds for the Z-axis PID controller
    private final double[] SPEED_GEARS = {0, 1};  // Speed gears used to adjust motor power

    // --- Hardware ---
    private DcMotor frontLeft, frontRight, backLeft, backRight;  // Motors for the mecanum wheels
    private DriveEncoder par1, par2, perp;  // Encoders for the motors
    private IMU imu;  // IMU for getting robot's orientation

    // --- OpMode & Config ---
    private LinearOpMode opMode;  // LinearOpMode object to interface with the robot's operation
    private final boolean IS_DEBUG_MODE;  // Flag for enabling/disabling debug mode

    // --- PID Controllers ---
    private PIDController xPid = new PIDController(0, 0, 0, 0);  // PID controller for X-axis movement
    private PIDController yPid = new PIDController(0, 0, 0, 0);  // PID controller for Y-axis movement
    private PIDController zPid = new PIDController(0, 0, 0, 0);  // PID controller for Z-axis (rotation) movement

    // --- Position Tracking ---
    private Pose2d fieldPos = new Pose2d(0, 0, 0);  // Robot's position on the field
    private Pose2d robotPos = new Pose2d(0, 0, 0);  // Robot's position relative to the robot's origin
    private Pose2d deltaFieldPos = new Pose2d(0, 0, 0);  // Change in position since the last update
    private Pose2d prevRawPos = new Pose2d(0, 0, 0);  // Previous raw position data
    private Pose2d startPos = new Pose2d(0, 0, 0);  // Starting position of the robot

    // --- Gear Setting ---
    private int gear = 0;  // Current gear setting for the robot

    // --- Constructor ---
    public MecanumDrive(LinearOpMode opMode, boolean isDebug) {
        this.opMode = opMode;  // Set the opMode for the robot
        this.IS_DEBUG_MODE = isDebug;  // Set debug mode flag

        // Initialize motors from hardware map
        frontLeft = opMode.hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = opMode.hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = opMode.hardwareMap.get(DcMotor.class, "backLeft");
        backRight = opMode.hardwareMap.get(DcMotor.class, "backRight");

        // Initialize encoders for the motors
        par1 = new DriveEncoder(frontLeft,  TICKS_PER_REV, SPOOL_DIAMETER);
        par2 = new DriveEncoder(frontRight, TICKS_PER_REV, SPOOL_DIAMETER);
        perp = new DriveEncoder(backRight,  TICKS_PER_REV, SPOOL_DIAMETER);

        // Initialize IMU
        imu = opMode.hardwareMap.get(IMU.class, "imu");

        init();  // Call the init method to configure the robot

        // Log a debug message indicating the subsystem has been initialized
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Drive Initialized");
    }

    // --- Init ---
    public void init() {
        // Set motor directions to match the robot's configuration
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        // Set motor behaviors for when the power is zero (brake mode)
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set PID tolerances and integration bounds
        xPid.setTolerance(X_TOLERANCE);
        yPid.setTolerance(Y_TOLERANCE);
        zPid.setTolerance(Z_TOLERANCE);

        xPid.setIntegrationBounds(INTEGRAL_BOUNDS_X[0], INTEGRAL_BOUNDS_X[1]);
        yPid.setIntegrationBounds(INTEGRAL_BOUNDS_Y[0], INTEGRAL_BOUNDS_Y[1]);
        zPid.setIntegrationBounds(INTEGRAL_BOUNDS_Z[0], INTEGRAL_BOUNDS_Z[1]);

        resetEncoders();  // Reset all the encoders to their initial positions
    }

    // --- Encoder Reset ---
    public void resetEncoders() {
        // Reset the encoders for all motors
        par1.resetEncoder();
        par2.resetEncoder();
        perp.resetEncoder();

        // Log a debug message indicating that encoders have been reset
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Encoders Reset");
    }

    // --- Odometry & Sensor Updates ---
    public void update() {
        // Update position by getting raw encoder data and IMU angle
        double x = getRawX();
        double y = getRawY();
        double angle = getAngle();

        // Update robot's position based on new sensor data
        robotPos.setAngle(angle);
        fieldPos.setAngle(angle + startPos.getAngle());

        deltaFieldPos.setX(getRawX() - prevRawPos.getX());
        deltaFieldPos.setY(getRawY() - prevRawPos.getY());
        deltaFieldPos.setAngle(angle - prevRawPos.getAngle());
        deltaFieldPos.rotateByDegrees(-deltaFieldPos.getAngle());

        fieldPos.setX(fieldPos.getX() + deltaFieldPos.getX() + startPos.getX());
        fieldPos.setY(fieldPos.getX() + deltaFieldPos.getY() + startPos.getY());

        robotPos = fieldPos.copy();
        robotPos.rotateByDegrees(angle);

        // Update previous raw position
        prevRawPos.setX(x);
        prevRawPos.setY(y);
        prevRawPos.setAngle(angle);

        // Log debugging information about the robot's position
        DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, new Object[][]{
                {"deltaX", deltaFieldPos.getX()},
                {"deltaY", deltaFieldPos.getY()},
                {"deltaAngle", deltaFieldPos.getAngle()},
                {"posX", fieldPos.getX()},
                {"posY", fieldPos.getY()},
                {"angle", fieldPos.getAngle()}
        });
    }

    // --- Get Angle ---
    public double getAngle() {
        // Return the robot's yaw (rotation) from the IMU
        return -MathUtil.normalizeAngleTo180(imu.getRobotYawPitchRollAngles().getYaw());
    }

    // --- Get Raw X ---
    public double getRawX() {
        // Return the raw X position from one of the encoders
        return perp.getPosition();
    }

    // --- Get Raw Y ---
    public double getRawY() {
        // Return the average of two encoders (front left and right)
        return (par1.getPosition() + par2.getPosition()) / 2;
    }

    // --- Set Motor Power ---
    public void setPower(Pose2d power) {
        // Set motor power values based on input X, Y, Z
        double x = power.getX();
        double y = power.getY();
        double z = power.getAngle();

        // Calculate denominator to normalize power values
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(z), 1);

        // Calculate individual motor power values for mecanum drive
        double rightFrontPower = (y - x - z) / denominator;
        double rightBackPower = (y + x - z) / denominator;
        double leftFrontPower = (y + x + z) / denominator;
        double leftBackPower = (y - x + z) / denominator;

        // Set power to motors
        frontLeft.setPower(leftFrontPower);
        backLeft.setPower(leftBackPower);
        frontRight.setPower(rightFrontPower);
        backRight.setPower(rightBackPower);

        // Log debug information for motor power values
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

    // --- Set Power By Gear ---
    public void setPowerByGear(Pose2d power) {
        // Apply gear adjustment to power values
        power.setX(getRawX() * SPEED_GEARS[gear]);
        power.setY(getRawY() * SPEED_GEARS[gear]);
        power.setAngle(getAngle() * SPEED_GEARS[gear]);
        setPower(power);  // Set the calculated power
    }

    // --- Field Drive ---
    public void fieldDrive(Pose2d power) {
        // Apply deadzone to input values to avoid unnecessary small movements
        power.setX(Math.pow(MathUtil.applyDeadzone(power.getX(), 0.1), 2));
        power.setY(Math.pow(MathUtil.applyDeadzone(power.getY(), 0.1), 2));
        power.setAngle(Math.pow(MathUtil.applyDeadzone(power.getAngle(), 0.1), 2));

        // Rotate the input power based on the robot's current angle
        power.rotateByDegrees(-getAngle());
        setPower(power);  // Apply the rotated power to the motors

        // Log debugging information for field drive
        DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Field Drive Power", power);
    }

    // --- Field Gear Drive ---
    public void fieldGearDrive(Pose2d power) {
        // Adjust for robot's angle and apply the gear adjustment
        power.rotateByDegrees(-getAngle());
        setPowerByGear(power);
        DebugUtils.logDebug(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Field Drive Power", power);
    }

    // --- Stop the Robot ---
    public void stop() {
        // Stop all motors
        setPower(new Pose2d(0, 0, 0));
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Motors Stopped");
    }

    // --- PID Drive ---
    public void pidDrive(Pose2d pose, double timeOut) {
        // Rotate the target pose based on robot's current angle
        pose.rotateByDegrees(-pose.getAngle());
        pose.rotateByDegrees(getAngle());

        // Set PID controller timeouts
        xPid.setTimeout(timeOut);
        yPid.setTimeout(timeOut);
        zPid.setTimeout(timeOut);

        // Reset PID controllers
        xPid.reset();
        yPid.reset();
        zPid.reset();

        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "Starting PID Drive");

        // Start the PID control loop
        do {
            update();

            // Calculate the power for each axis based on PID
            double xPower = xPid.calculate(robotPos.getX(), pose.getX());
            double yPower = yPid.calculate(robotPos.getY(), pose.getY());
            double zPower = zPid.calculate(robotPos.getAngle(), pose.getAngle());

            // Set the calculated power to the motors
            setPower(new Pose2d(xPower, yPower, zPower));

            // Log debugging information during PID control
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

        stop();  // Stop the robot once the PID control loop is done
        DebugUtils.logDebugMessage(opMode.telemetry, IS_DEBUG_MODE, SUBSYSTEM_NAME, "PID Drive Complete");
    }

    // --- Getters / Setters ---
    public int getGear() {
        return gear;  // Return current gear setting
    }

    public void setGear(int gear) {
        gear = (int) MathUtil.clamp(gear, 0, SPEED_GEARS.length - 1);  // Clamp gear value within range
        this.gear = gear;  // Set new gear value
    }

    public void setStartPos(Pose2d startPos) {
        this.startPos = startPos;  // Set the starting position
    }
}
