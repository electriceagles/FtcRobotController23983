package org.firstinspires.ftc.teamcode.basicOpMode.Tea_andEthan_Auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.Range;
import android.util.Size;


import androidx.annotation.Nullable;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*@Disabled Disabled it for now since I cannot test it, If someone does plan to test this
Record a video of the test and remove the @Disabled and remember to push code back to the control hub
 */
@Autonomous
public class EncoderAutoRed extends LinearOpMode {

    public RobotHardware hardware = new RobotHardware();

    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTag;

    public DcMotorEx turret;

    public static final int TARGET_TAG_ID = 24; //  24 for red
    public static final double SCAN_POWER = 0.30;
    public static final double SCAN_FREQ  = 0.25;

    public static final double TURRET_KP = 0.02;          // power per degree of yaw (start small)
    public static final double MAX_TURRET_POWER = 0.35;
    public static final double YAW_TOL_DEG = 2.0;
    public static final double TICKS_PER_REV = 383.6;
    public static final double WHEEL_DIAMETER = 4.09;  // Inches
    public static final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER);

    public static final double TRACK_WIDTH = 12; // Inches

    @Override
    public void runOpMode() throws InterruptedException {

//        /** even tho this is auto, I don't want to use roadrunner for this auto **/
        hardware.init(hardwareMap, false);
        hardware.resetEnc();

        aprilTag = new AprilTagProcessor.Builder().build();

        WebcamName webcam = hardwareMap.get(WebcamName.class, "webcam");

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .addProcessor(aprilTag)
                .build();

        waitForStart();


        if (opModeIsActive()) {

            // This is a test if this works we can have this as a backup auto in case you don't have enough time to tune.
            boolean seen = scanTurretUntilTagSeen(2.5);
            telemetry.addData("Scan result", seen ? "FOUND" : "NOT FOUND");
            boolean centered = trackTurretToCenter(1.5);
            telemetry.addData("Center result", centered ? "CENTERED" : "NOT CENTERED");
            telemetry.update();

            if (centered) {
                driveInches(72, 0.25);
                turn(90, 0.5);
                rev();
                intake();
                rev();
                strafeInches("right", 24,0.5);
                intake();
                strafeInches("left",24,0.5);
                rev();
                hardware.turret.setPower(0);
                if (visionPortal != null) visionPortal.close();
            }
            else {
                driveInches(10,0.5);
            }
        }

    }
    @Nullable
    private AprilTagDetection getTargetTag() {
        if (aprilTag == null) return null;

        List<AprilTagDetection> detections = aprilTag.getDetections();
        if (detections == null) return null;

        for (AprilTagDetection tag : detections) {
            if (tag.id == TARGET_TAG_ID) return tag;
        }
        return null;
    }

    public boolean scanTurretUntilTagSeen(double timeoutSeconds) {
        double start = getRuntime();

        while (opModeIsActive() && (getRuntime() - start) < timeoutSeconds) {
            AprilTagDetection target = getTargetTag();

            telemetry.addData("Tag Found?", target != null);
            if (target != null) {
                telemetry.addData("Tag ID", target.id);
                telemetry.update();
                hardware.turret.setPower(0);
                return true; // found it
            }

            double t = getRuntime() - start;
            double s = Math.sin(2 * Math.PI * SCAN_FREQ * t);
            hardware.turret.setPower(SCAN_POWER * Math.signum(s));

            telemetry.addData("Scanning", "YES");
            telemetry.addData("Elapsed", "%.2f / %.2f", (getRuntime() - start), timeoutSeconds);
            telemetry.update();

            idle(); // important so vision + system threads run
        }

        hardware.turret.setPower(0);
        telemetry.addData("Tag Found?", false);
        telemetry.addData("Result", "Timed out");
        telemetry.update();
        return false; // timed out
    }
    public boolean trackTurretToCenter(double timeoutSeconds) {
        double start = getRuntime();

        while (opModeIsActive() && (getRuntime() - start) < timeoutSeconds) {
            AprilTagDetection target = getTargetTag();
            if (target == null) {
                hardware.turret.setPower(0);
                return false;
            }

            double yaw = target.ftcPose.yaw; // degrees (positive/negative depends on camera orientation)

            if (Math.abs(yaw) <= YAW_TOL_DEG) {
                hardware.turret.setPower(0);
                return true; // centered enough
            }

            double power = Range.clip(TURRET_KP * yaw, -MAX_TURRET_POWER, MAX_TURRET_POWER);

            // If it turns the wrong direction, flip the sign:
            hardware.turret.setPower(power);

            idle();
        }

        hardware.turret.setPower(0);
        return false;
    }
    public void intake() {
        hardware.intake.setPower(0.67);
        driveInches(36, 0.5);
        hardware.intake.setPower(0);
        sleep(500);
        driveInches(36, -0.5);
    }
    public void driveInches(double inches, double power) {
        int ticks = (int) Math.round(inches * TICKS_PER_INCH);
        drive(ticks, power);
    }
    private void waitForDriveMotors() {
        while (opModeIsActive()
                && (hardware.leftFront.isBusy()
                || hardware.rightFront.isBusy()
                || hardware.leftRear.isBusy()
                || hardware.rightRear.isBusy())) {
            idle();
        }

        // stop motors after reaching target
        hardware.leftFront.setPower(0);
        hardware.rightFront.setPower(0);
        hardware.leftRear.setPower(0);
        hardware.rightRear.setPower(0);
    }

    public void rev(){
        hardware.resetEnc();

        hardware.shooterFlyWheel1.setPower(0.95);
        hardware.shooterFlyWheel2.setPower(0.95);

        sleep((3500));
        hardware.servo.setPosition(0);

        sleep((3500));
        hardware.shooterFlyWheel1.setPower(0);
        hardware.shooterFlyWheel2.setPower(0);
        hardware.servo.setPosition(0.5);
        sleep(1000);
    }

    public void drive(int encPos, double power) {
        hardware.resetEnc();

        hardware.leftFront.setTargetPosition(encPos);
        hardware.rightFront.setTargetPosition(encPos);
        hardware.rightRear.setTargetPosition(encPos);
        hardware.leftRear.setTargetPosition(encPos);

        hardware.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hardware.leftFront.setPower(power);
        hardware.rightFront.setPower(power);
        hardware.rightRear.setPower(power);
        hardware.leftRear.setPower(power);

        waitForDriveMotors();
    }

    public void turn(int degrees, double power) {
        hardware.resetEnc();
        double turnInches = Math.PI * TRACK_WIDTH * (degrees / 360.0);

        int turnTicks = (int) (turnInches * TICKS_PER_INCH);

        hardware.leftRear.setTargetPosition(hardware.leftRear.getCurrentPosition() + turnTicks);
        hardware.rightRear.setTargetPosition(hardware.rightRear.getCurrentPosition() - turnTicks);
        hardware.leftFront.setTargetPosition(hardware.leftFront.getCurrentPosition() + turnTicks);
        hardware.rightFront.setTargetPosition(hardware.rightFront.getCurrentPosition() - turnTicks);

        hardware.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hardware.leftFront.setPower(power);
        hardware.rightFront.setPower(power);
        hardware.rightRear.setPower(power);
        hardware.leftRear.setPower(power);

        waitForDriveMotors();
    }
    public void strafeInches(String direction, double inches, double power) {
        int ticks = (int) Math.round(inches * TICKS_PER_INCH);
        strafe(direction, ticks, power);
        waitForDriveMotors();
    }

    public void strafe(String direction, int encPos, double power) {
        hardware.resetEnc();

        hardware.leftFront.setTargetPosition(encPos);
        hardware.rightFront.setTargetPosition(encPos);
        hardware.rightRear.setTargetPosition(encPos);
        hardware.leftRear.setTargetPosition(encPos);

        hardware.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (direction.equals("left")) {
            hardware.leftFront.setPower(-power);
            hardware.rightFront.setPower(power);
            hardware.rightRear.setPower(-power);
            hardware.leftRear.setPower(power);
        } else if (direction.equals("right")) {
            hardware.leftFront.setPower(power);
            hardware.rightFront.setPower(-power);
            hardware.rightRear.setPower(power);
            hardware.leftRear.setPower(-power);

        }
    }
}
