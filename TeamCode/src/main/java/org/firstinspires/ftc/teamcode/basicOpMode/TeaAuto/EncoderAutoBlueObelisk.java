package org.firstinspires.ftc.teamcode.basicOpMode.TeaAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

/**
 * EncoderAutoBlueObelisk demonstrates encoder-based autonomous movement.
 *
 * <p>This is a simpler alternative to RoadRunner that uses the built-in
 * motor encoder functionality. Good for teams that want consistent
 * movements without the complexity of path planning.</p>
 *
 * <p><b>Note:</b> This OpMode is currently disabled. To test it:
 * 1. Remove the @Disabled annotation
 * 2. Push to the Control Hub
 * 3. Record a video of the test results</p>
 */
@Disabled
@Autonomous(name = "Encoder Auto Blue Obelisk", group = "Auto")
public class EncoderAutoBlueObelisk extends LinearOpMode {

    public RobotHardware hardware = new RobotHardware();

    // Motor encoder constants - adjust for your specific motors
    public static final double TICKS_PER_REV = 383.6;  // GoBilda 435 RPM motor
    public static final double WHEEL_DIAMETER = 4.09;  // Inches (mecanum wheels)
    public static final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER);

    public static final double TRACK_WIDTH = 12;  // Inches - distance between wheels

    // Timeout for encoder movements (prevents infinite loops if motor stalls)
    public static final double MOVEMENT_TIMEOUT = 5.0;  // seconds

    // 30-second autonomous safety limit
    private ElapsedTime runtime = new ElapsedTime();
    private static final double AUTO_TIME_LIMIT = 29.5;

    @Override
    public void runOpMode() throws InterruptedException {

        // Initialize hardware WITHOUT RoadRunner (pass false)
        hardware.init(hardwareMap, false);
        hardware.resetEnc();

        // Set motors to brake mode for precise stopping
        setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Encoder Auto Blue Obelisk");
        telemetry.addLine("Status: Initialized");
        telemetry.addLine("Waiting for START...");
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (opModeIsActive()) {

            // Example autonomous sequence - adjust distances for your field
            telemetry.addLine(">>> Driving forward");
            telemetry.update();
            drive(24, 0.5);  // Drive forward 24 inches

            if (shouldStop()) return;

            telemetry.addLine(">>> Turning right 90 degrees");
            telemetry.update();
            turn(90, 0.5);   // Turn right 90 degrees

            if (shouldStop()) return;

            telemetry.addLine(">>> Strafing left");
            telemetry.update();
            strafe(-12, 0.5);  // Strafe left 12 inches (negative = left)

            telemetry.addLine("=== Autonomous Complete ===");
            telemetry.addData("Total Time", "%.1f sec", runtime.seconds());
            telemetry.update();
        }
    }

    /**
     * Checks if autonomous should stop (timeout or user request).
     */
    private boolean shouldStop() {
        return isStopRequested() || runtime.seconds() >= AUTO_TIME_LIMIT;
    }

    /**
     * Sets zero power behavior for all drive motors.
     */
    private void setZeroPowerBehavior(DcMotor.ZeroPowerBehavior behavior) {
        hardware.leftFront.setZeroPowerBehavior(behavior);
        hardware.leftRear.setZeroPowerBehavior(behavior);
        hardware.rightFront.setZeroPowerBehavior(behavior);
        hardware.rightRear.setZeroPowerBehavior(behavior);
    }

    /**
     * Drives forward or backward a specified distance using encoders.
     *
     * @param inches   distance to travel (positive = forward, negative = backward)
     * @param power    motor power (0.0 to 1.0)
     */
    public void drive(double inches, double power) {
        if (shouldStop()) return;

        int targetTicks = (int) (inches * TICKS_PER_INCH);
        power = Math.abs(power) * Math.signum(inches);

        hardware.resetEnc();

        // All four motors move the same direction for forward/backward
        hardware.leftFront.setTargetPosition(targetTicks);
        hardware.rightFront.setTargetPosition(targetTicks);
        hardware.rightRear.setTargetPosition(targetTicks);
        hardware.leftRear.setTargetPosition(targetTicks);

        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        hardware.leftFront.setPower(power);
        hardware.rightFront.setPower(power);
        hardware.rightRear.setPower(power);
        hardware.leftRear.setPower(power);

        // Wait for motors to reach target (with timeout)
        waitForMotors();

        stopMotors();
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Turns the robot in place by a specified angle using encoders.
     *
     * @param degrees  angle to turn (positive = right, negative = left)
     * @param power    motor power (0.0 to 1.0)
     */
    public void turn(double degrees, double power) {
        if (shouldStop()) return;

        // Calculate arc length each wheel must travel
        double turnInches = Math.PI * TRACK_WIDTH * (degrees / 360.0);
        int turnTicks = (int) (Math.abs(turnInches) * TICKS_PER_INCH);
        int direction = degrees > 0 ? 1 : -1;  // positive = right turn

        hardware.resetEnc();

        // For turning: left wheels and right wheels go opposite directions
        // Turn right: left forward (+), right backward (-)
        hardware.leftFront.setTargetPosition(turnTicks * direction);
        hardware.leftRear.setTargetPosition(turnTicks * direction);
        hardware.rightFront.setTargetPosition(-turnTicks * direction);
        hardware.rightRear.setTargetPosition(-turnTicks * direction);

        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        hardware.leftFront.setPower(power);
        hardware.rightFront.setPower(power);
        hardware.rightRear.setPower(power);
        hardware.leftRear.setPower(power);

        waitForMotors();

        stopMotors();
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Strafes left or right using mecanum drive with encoders.
     *
     * <p>For mecanum strafing, diagonal wheel pairs move in the same direction:
     * <ul>
     *   <li>Strafe RIGHT: LF+, LR-, RF-, RR+</li>
     *   <li>Strafe LEFT:  LF-, LR+, RF+, RR-</li>
     * </ul></p>
     *
     * @param inches   distance to strafe (positive = right, negative = left)
     * @param power    motor power (0.0 to 1.0)
     */
    public void strafe(double inches, double power) {
        if (shouldStop()) return;

        int targetTicks = (int) (Math.abs(inches) * TICKS_PER_INCH);
        int direction = inches > 0 ? 1 : -1;  // positive = right

        hardware.resetEnc();

        // Mecanum strafe pattern: diagonal pairs move together
        // Strafe RIGHT: LF forward, LR backward, RF backward, RR forward
        hardware.leftFront.setTargetPosition(targetTicks * direction);
        hardware.leftRear.setTargetPosition(-targetTicks * direction);
        hardware.rightFront.setTargetPosition(-targetTicks * direction);
        hardware.rightRear.setTargetPosition(targetTicks * direction);

        setRunMode(DcMotor.RunMode.RUN_TO_POSITION);

        hardware.leftFront.setPower(power);
        hardware.rightFront.setPower(power);
        hardware.rightRear.setPower(power);
        hardware.leftRear.setPower(power);

        waitForMotors();

        stopMotors();
        setRunMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    /**
     * Sets the run mode for all drive motors.
     */
    private void setRunMode(DcMotor.RunMode mode) {
        hardware.leftFront.setMode(mode);
        hardware.leftRear.setMode(mode);
        hardware.rightFront.setMode(mode);
        hardware.rightRear.setMode(mode);
    }

    /**
     * Waits for all drive motors to reach their target position.
     * Includes timeout protection to prevent infinite loops.
     */
    private void waitForMotors() {
        ElapsedTime moveTimer = new ElapsedTime();

        while (opModeIsActive() &&
               !shouldStop() &&
               moveTimer.seconds() < MOVEMENT_TIMEOUT &&
               (hardware.leftFront.isBusy() || hardware.leftRear.isBusy() ||
                hardware.rightFront.isBusy() || hardware.rightRear.isBusy())) {

            // Display progress
            telemetry.addData("LF", "%d / %d",
                    hardware.leftFront.getCurrentPosition(),
                    hardware.leftFront.getTargetPosition());
            telemetry.addData("RF", "%d / %d",
                    hardware.rightFront.getCurrentPosition(),
                    hardware.rightFront.getTargetPosition());
            telemetry.addData("LR", "%d / %d",
                    hardware.leftRear.getCurrentPosition(),
                    hardware.leftRear.getTargetPosition());
            telemetry.addData("RR", "%d / %d",
                    hardware.rightRear.getCurrentPosition(),
                    hardware.rightRear.getTargetPosition());
            telemetry.update();

            sleep(20);
        }
    }

    /**
     * Stops all drive motors.
     */
    private void stopMotors() {
        hardware.leftFront.setPower(0);
        hardware.leftRear.setPower(0);
        hardware.rightFront.setPower(0);
        hardware.rightRear.setPower(0);
    }
}
