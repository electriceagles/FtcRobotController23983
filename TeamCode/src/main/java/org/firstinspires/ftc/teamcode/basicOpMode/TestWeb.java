package org.firstinspires.ftc.teamcode.basicOpMode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Q2 TeleOp maybe", group = "TeleOp")
public class TestWeb extends LinearOpMode {

    // ================= DRIVE =================
    DcMotorEx lf, lr, rf, rr;
    double powerMult = 0.9;

    // ================= MECHANISMS =================
    DcMotorEx intake;
    DcMotorEx turret;
    DcMotorEx shooter1, shooter2;
    Servo servo;

    // ================= VISION =================
    VisionPortal visionPortal;
    AprilTagProcessor aprilTag;

    static final int TARGET_TAG_ID = 20;

    // ================= TURRET CONSTANTS =================
    static final double TICKS_PER_REV = 537.7;   // goBILDA 5202
    static final double GEAR_RATIO = 5.0;
    static final double TICKS_PER_DEGREE =
            (TICKS_PER_REV * GEAR_RATIO) / 360.0;

    static final double TARGET_YAW = 0.0;
    static final double YAW_TOLERANCE = 0.6;

    static final double SCAN_POWER = 0.4;
    static final double SCAN_RANGE_DEG = 60.0;
    static final double SCAN_RANGE_TICKS =
            SCAN_RANGE_DEG * TICKS_PER_DEGREE;

    double scanCenterTicks = 0;

    // ================= PID =================
    double Kp = 0.008;
    double Ki = 0.0;
    double Kd = 0.001;

    double integralSum = 0;
    double lastError = 0;
    ElapsedTime pidTimer = new ElapsedTime();

    boolean turretAuto = false;
    boolean manControl = true;

    // ================= SHOOTER VELOCITIES =================
    static final double RPM_3000 = 1400;
    static final double RPM_3200 = 1493;
    static final double RPM_3900 = 1820;
    static final double RPM_4700 = 2193;

    @Override
    public void runOpMode() {

        // ---------- Drive ----------
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lr = hardwareMap.get(DcMotorEx.class, "lr");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rr = hardwareMap.get(DcMotorEx.class, "rr");

        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lr.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        // ---------- Mechanisms ----------
        intake = hardwareMap.get(DcMotorEx.class, "i");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        shooter1 = hardwareMap.get(DcMotorEx.class, "sf1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "sf2");
        servo = hardwareMap.get(Servo.class, "servo");

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ---------- Vision ----------
        aprilTag = new AprilTagProcessor.Builder().build();
        WebcamName webcam = hardwareMap.get(WebcamName.class, "webcam");

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .addProcessor(aprilTag)
                .build();

        visionPortal.setProcessorEnabled(aprilTag, true);

        waitForStart();

        // ==================================================
        while (opModeIsActive()) {

            // ================= DRIVE =================
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            if (Math.abs(y) < 0.05) y = 0;
            if (Math.abs(x) < 0.05) x = 0;
            if (Math.abs(rx) < 0.05) rx = 0;

            lf.setPower((y + x + rx) * powerMult);
            rf.setPower((y - x - rx) * powerMult);
            lr.setPower((y - x + rx) * powerMult);
            rr.setPower((y + x - rx) * powerMult);

            // ================= SHOOTER RPM =================
            double targetVelocity = 0;

            if (gamepad1.right_bumper) {
                targetVelocity = RPM_3200;
            } else if (gamepad1.triangle) {
                targetVelocity = RPM_3000;
            } else if (gamepad1.square) {
                targetVelocity = RPM_3900;
            } else if (gamepad1.cross) {
                targetVelocity = RPM_4700;
            }

            if (targetVelocity > 0) {
                shooter1.setVelocity(targetVelocity);
                shooter2.setVelocity(targetVelocity);
            } else {
                shooter1.setPower(0);
                shooter2.setPower(0);
            }

            // ================= FEED =================
            if (gamepad1.right_trigger > 0.1) {
                servo.setPosition(1.0); // feed
            } else {
                servo.setPosition(0.0); // retract
            }

            // ================= INTAKE =================
            if (gamepad1.left_trigger > 0.1) {
                intake.setPower(1);
            } else if (gamepad1.left_bumper) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            // ================= AUTO START =================
            if (gamepad1.dpadUpWasPressed()) {
                turretAuto = true;
                manControl = false;
                integralSum = 0;
                lastError = 0;
                pidTimer.reset();
                scanCenterTicks = turret.getCurrentPosition();
            }

            // ================= TURRET AUTO =================
            if (turretAuto) {

                List<AprilTagDetection> detections = aprilTag.getDetections();
                AprilTagDetection targetTag = null;

                for (AprilTagDetection tag : detections) {
                    if (tag.id == TARGET_TAG_ID) {
                        targetTag = tag;
                        break;
                    }
                }

                if (targetTag != null) {

                    double yaw = targetTag.ftcPose.yaw;
                    double yawError = TARGET_YAW - yaw;

                    double targetTicks =
                            turret.getCurrentPosition()
                                    + (yawError * TICKS_PER_DEGREE);

                    double error = targetTicks - turret.getCurrentPosition();

                    double dt = pidTimer.seconds();
                    if (dt <= 0) dt = 1e-3;
                    pidTimer.reset();

                    integralSum += error * dt;
                    double derivative = (error - lastError) / dt;

                    double output =
                            (Kp * error) +
                                    (Ki * integralSum) +
                                    (Kd * derivative);

                    output = Math.max(-0.5, Math.min(0.5, output));
                    turret.setPower(output);
                    lastError = error;

                    if (Math.abs(yaw) < YAW_TOLERANCE) {
                        turret.setPower(0);
                        turretAuto = false;
                        manControl = true;
                    }

                } else {
                    double pos = turret.getCurrentPosition();
                    double leftLimit = scanCenterTicks - SCAN_RANGE_TICKS;
                    double rightLimit = scanCenterTicks + SCAN_RANGE_TICKS;

                    if (SCAN_POWER > 0 && pos >= rightLimit) {
                        turret.setPower(0);
                    } else {
                        turret.setPower(SCAN_POWER);
                    }
                }

            }

            // ================= MANUAL TURRET =================
            else if (manControl) {
                if (gamepad1.dpad_left) {
                    turret.setPower(0.8);
                } else if (gamepad1.dpad_right) {
                    turret.setPower(-0.8);
                } else {
                    turret.setPower(0);
                }
            }
        }
    }
}