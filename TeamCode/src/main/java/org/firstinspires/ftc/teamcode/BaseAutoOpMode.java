package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;

/**
 * BaseAutoOpMode provides common helper methods for timed and encoder-based
 * driving during autonomous routines. Subclasses can call these methods to
 * build sequences without duplicating low-level code.
 *
 * <p>This class uses the team's existing Hardware.RobotHardware configuration
 * which maps motors to "lf", "lr", "rf", "rr", "i", "sf1", "sf2", "turret".</p>
 *
 * <p><b>IMPORTANT:</b> The autonomous period is 30 seconds. All methods check
 * {@link #shouldStop()} to ensure the robot stops before time expires.</p>
 */
public abstract class BaseAutoOpMode extends LinearOpMode {

    /** Hardware abstraction - uses existing team configuration. */
    protected RobotHardware robot = new RobotHardware();

    /** Timer for tracking elapsed autonomous time. */
    protected ElapsedTime runtime = new ElapsedTime();

    /** Maximum autonomous duration in seconds (with safety margin). */
    protected static final double AUTO_TIME_LIMIT = 29.5;

    // Encoder constants (from DriveConstants)
    protected static final double TICKS_PER_REV = DriveConstants.TICKS_PER_REV;
    protected static final double WHEEL_DIAMETER_INCHES = DriveConstants.WHEEL_RADIUS * 2;
    protected static final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER_INCHES);
    protected static final double TRACK_WIDTH = DriveConstants.TRACK_WIDTH;

    /**
     * Checks if the autonomous routine should stop (either user requested stop
     * or 30-second time limit approaching).
     *
     * @return true if the robot should stop all actions
     */
    protected boolean shouldStop() {
        return isStopRequested() || runtime.seconds() >= AUTO_TIME_LIMIT;
    }

    /**
     * Initializes hardware and sets motors to RUN_WITHOUT_ENCODER mode for
     * time-based control. Call this at the start of runOpMode().
     */
    protected void initHardware() {
        robot.init(hardwareMap, false);
        setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Sets the run mode for all drive motors.
     *
     * @param mode the DcMotor.RunMode to set
     */
    protected void setDriveRunMode(DcMotor.RunMode mode) {
        robot.leftFront.setMode(mode);
        robot.leftRear.setMode(mode);
        robot.rightFront.setMode(mode);
        robot.rightRear.setMode(mode);
    }

    /**
     * Resets all drive motor encoders to zero.
     */
    protected void resetDriveEncoders() {
        robot.resetEnc();
    }

    // ========================================================================
    // TIME-BASED MOVEMENT METHODS
    // ========================================================================

    /**
     * Drives the robot by setting left and right motor power for a fixed
     * duration. Positive power drives forward, negative power drives backward.
     *
     * @param leftPower    power applied to the left motors (-1.0 to 1.0)
     * @param rightPower   power applied to the right motors (-1.0 to 1.0)
     * @param milliseconds duration in milliseconds
     */
    protected void moveForTime(double leftPower, double rightPower, long milliseconds) {
        if (shouldStop()) return;

        robot.leftFront.setPower(leftPower);
        robot.leftRear.setPower(leftPower);
        robot.rightFront.setPower(rightPower);
        robot.rightRear.setPower(rightPower);

        ElapsedTime moveTimer = new ElapsedTime();
        while (opModeIsActive() && !shouldStop() && moveTimer.milliseconds() < milliseconds) {
            telemetry.addData("Move", "%.1f / %d ms", moveTimer.milliseconds(), milliseconds);
            telemetry.update();
            sleep(20);
        }

        stopDrive();
    }

    /**
     * Strafes the robot left or right using mecanum drive. The direction
     * parameter should be 1 for right or -1 for left.
     *
     * @param power         speed to strafe (0.0 to 1.0)
     * @param milliseconds  duration of the strafe
     * @param directionSign 1 for right, -1 for left
     */
    protected void strafeForTime(double power, long milliseconds, int directionSign) {
        if (shouldStop()) return;

        // Mecanum strafing: diagonal pairs run together
        // Strafe RIGHT: LF+, LR-, RF-, RR+
        // Strafe LEFT:  LF-, LR+, RF+, RR-
        robot.leftFront.setPower(power * directionSign);
        robot.leftRear.setPower(-power * directionSign);
        robot.rightFront.setPower(-power * directionSign);
        robot.rightRear.setPower(power * directionSign);

        ElapsedTime strafeTimer = new ElapsedTime();
        while (opModeIsActive() && !shouldStop() && strafeTimer.milliseconds() < milliseconds) {
            telemetry.addData("Strafe", "%.1f / %d ms", strafeTimer.milliseconds(), milliseconds);
            telemetry.update();
            sleep(20);
        }

        stopDrive();
    }

    /**
     * Turns the robot in place. Positive power turns right, negative turns left.
     *
     * @param power        turning power (-1.0 to 1.0)
     * @param milliseconds duration of the turn
     */
    protected void turnForTime(double power, long milliseconds) {
        if (shouldStop()) return;

        // Turn right: left motors forward, right motors backward
        robot.leftFront.setPower(power);
        robot.leftRear.setPower(power);
        robot.rightFront.setPower(-power);
        robot.rightRear.setPower(-power);

        ElapsedTime turnTimer = new ElapsedTime();
        while (opModeIsActive() && !shouldStop() && turnTimer.milliseconds() < milliseconds) {
            telemetry.addData("Turn", "%.1f / %d ms", turnTimer.milliseconds(), milliseconds);
            telemetry.update();
            sleep(20);
        }

        stopDrive();
    }

    // ========================================================================
    // ENCODER-BASED MOVEMENT METHODS
    // ========================================================================

    /**
     * Drives forward or backward using encoders for consistent distance.
     *
     * @param inches distance to travel (positive = forward, negative = backward)
     * @param power  motor power (0.0 to 1.0)
     */
    protected void driveDistance(double inches, double power) {
        if (shouldStop()) return;

        int targetTicks = (int) (inches * TICKS_PER_INCH);
        double direction = Math.signum(inches);
        power = Math.abs(power) * direction;

        resetDriveEncoders();

        robot.leftFront.setTargetPosition(targetTicks);
        robot.leftRear.setTargetPosition(targetTicks);
        robot.rightFront.setTargetPosition(targetTicks);
        robot.rightRear.setTargetPosition(targetTicks);

        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftFront.setPower(power);
        robot.leftRear.setPower(power);
        robot.rightFront.setPower(power);
        robot.rightRear.setPower(power);

        // Wait for motors to reach position
        while (opModeIsActive() && !shouldStop() && driveMotorsBusy()) {
            telemetry.addData("Drive Encoder", "Target: %d, LF: %d",
                    targetTicks, robot.leftFront.getCurrentPosition());
            telemetry.update();
            sleep(20);
        }

        stopDrive();
        setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Strafes left or right using encoders.
     *
     * @param inches distance to strafe (positive = right, negative = left)
     * @param power  motor power (0.0 to 1.0)
     */
    protected void strafeDistance(double inches, double power) {
        if (shouldStop()) return;

        int targetTicks = (int) (Math.abs(inches) * TICKS_PER_INCH);
        int direction = inches > 0 ? 1 : -1;  // positive = right

        resetDriveEncoders();

        // Mecanum strafe: diagonal wheels move same direction
        // Strafe RIGHT: LF+, LR-, RF-, RR+
        robot.leftFront.setTargetPosition(targetTicks * direction);
        robot.leftRear.setTargetPosition(-targetTicks * direction);
        robot.rightFront.setTargetPosition(-targetTicks * direction);
        robot.rightRear.setTargetPosition(targetTicks * direction);

        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftFront.setPower(power);
        robot.leftRear.setPower(power);
        robot.rightFront.setPower(power);
        robot.rightRear.setPower(power);

        while (opModeIsActive() && !shouldStop() && driveMotorsBusy()) {
            telemetry.addData("Strafe Encoder", "Target: %d", targetTicks);
            telemetry.update();
            sleep(20);
        }

        stopDrive();
        setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Turns a specified number of degrees using encoders.
     *
     * @param degrees angle to turn (positive = right, negative = left)
     * @param power   motor power (0.0 to 1.0)
     */
    protected void turnDegrees(double degrees, double power) {
        if (shouldStop()) return;

        // Calculate arc length for the turn
        double turnInches = Math.PI * TRACK_WIDTH * (degrees / 360.0);
        int targetTicks = (int) (Math.abs(turnInches) * TICKS_PER_INCH);
        int direction = degrees > 0 ? 1 : -1;  // positive = right

        resetDriveEncoders();

        // Turn right: left wheels forward, right wheels backward
        robot.leftFront.setTargetPosition(targetTicks * direction);
        robot.leftRear.setTargetPosition(targetTicks * direction);
        robot.rightFront.setTargetPosition(-targetTicks * direction);
        robot.rightRear.setTargetPosition(-targetTicks * direction);

        setDriveRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.leftFront.setPower(power);
        robot.leftRear.setPower(power);
        robot.rightFront.setPower(power);
        robot.rightRear.setPower(power);

        while (opModeIsActive() && !shouldStop() && driveMotorsBusy()) {
            telemetry.addData("Turn Encoder", "Target: %d deg", (int) degrees);
            telemetry.update();
            sleep(20);
        }

        stopDrive();
        setDriveRunMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Checks if any drive motor is still moving to its target position.
     *
     * @return true if at least one motor is busy
     */
    protected boolean driveMotorsBusy() {
        return robot.leftFront.isBusy() || robot.leftRear.isBusy() ||
               robot.rightFront.isBusy() || robot.rightRear.isBusy();
    }

    // ========================================================================
    // MECHANISM CONTROL METHODS
    // ========================================================================

    /**
     * Runs the intake motor for a specified duration. Positive power intakes
     * artifacts; negative power reverses the intake.
     *
     * @param power        intake motor power (-1.0 to 1.0)
     * @param milliseconds duration to run the intake
     */
    protected void runIntakeForTime(double power, long milliseconds) {
        if (shouldStop()) return;

        robot.intake.setPower(power);

        ElapsedTime intakeTimer = new ElapsedTime();
        while (opModeIsActive() && !shouldStop() && intakeTimer.milliseconds() < milliseconds) {
            telemetry.addData("Intake", "%.1f / %d ms", intakeTimer.milliseconds(), milliseconds);
            telemetry.update();
            sleep(20);
        }

        robot.intake.setPower(0);
    }

    /**
     * Runs the shooter flywheels to deposit artifacts. The robot uses dual
     * flywheels (sf1, sf2) to shoot artifacts into the classifier.
     *
     * @param power        shooter power (0.0 to 1.0)
     * @param milliseconds duration to run the shooter
     */
    protected void runShooterForTime(double power, long milliseconds) {
        if (shouldStop()) return;

        robot.shooterFlyWheel1.setPower(power);
        robot.shooterFlyWheel2.setPower(power);

        ElapsedTime shooterTimer = new ElapsedTime();
        while (opModeIsActive() && !shouldStop() && shooterTimer.milliseconds() < milliseconds) {
            telemetry.addData("Shooter", "%.1f / %d ms", shooterTimer.milliseconds(), milliseconds);
            telemetry.update();
            sleep(20);
        }

        robot.shooterFlyWheel1.setPower(0);
        robot.shooterFlyWheel2.setPower(0);
    }

    /**
     * Spins up the shooter flywheels without blocking. Use this to pre-spin
     * the shooter before reaching the scoring position.
     *
     * @param power shooter power (0.0 to 1.0)
     */
    protected void startShooter(double power) {
        robot.shooterFlyWheel1.setPower(power);
        robot.shooterFlyWheel2.setPower(power);
    }

    /**
     * Stops the shooter flywheels.
     */
    protected void stopShooter() {
        robot.shooterFlyWheel1.setPower(0);
        robot.shooterFlyWheel2.setPower(0);
    }

    /**
     * Deposits artifacts by running intake in reverse to feed into shooter,
     * while shooter flywheels are spinning. This is the typical artifact
     * scoring sequence.
     *
     * @param shooterPower power for the shooter flywheels
     * @param intakePower  power for the intake (typically negative to feed)
     * @param milliseconds duration of the deposit sequence
     */
    protected void depositArtifacts(double shooterPower, double intakePower, long milliseconds) {
        if (shouldStop()) return;

        // Spin up shooter first
        startShooter(shooterPower);
        sleep(500);  // Let flywheels reach speed

        // Feed artifacts through
        robot.intake.setPower(intakePower);

        ElapsedTime depositTimer = new ElapsedTime();
        while (opModeIsActive() && !shouldStop() && depositTimer.milliseconds() < milliseconds) {
            telemetry.addData("Depositing", "%.1f / %d ms", depositTimer.milliseconds(), milliseconds);
            telemetry.update();
            sleep(20);
        }

        robot.intake.setPower(0);
        stopShooter();
    }

    /**
     * Stops all drive motors.
     */
    protected void stopDrive() {
        robot.leftFront.setPower(0);
        robot.leftRear.setPower(0);
        robot.rightFront.setPower(0);
        robot.rightRear.setPower(0);
    }

    /**
     * Stops all motors on the robot (drive, intake, shooter).
     */
    protected void stopAllMotors() {
        stopDrive();
        robot.intake.setPower(0);
        robot.shooterFlyWheel1.setPower(0);
        robot.shooterFlyWheel2.setPower(0);
        robot.turret.setPower(0);
    }
}
