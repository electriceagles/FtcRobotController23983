package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Turret AprilTag Aimer", group = "Vision")
public class TurretTagAimer extends LinearOpMode {

    // -------------------- TUNABLES --------------------
    // Control gains for turret (P + D; set KI=0 to start)
    private static final double KP   = 0.025;  // power per degree of yaw error
    private static final double KD   = 0.002;  // helps damp oscillation (power per deg/sec)
    private static final double KI   = 0.000;  // optional integral if you see steady-state error
    private static final double KS   = 0.05;   // static feedforward to overcome friction (applied with sign of command)

    private static final double MAX_POWER      = 0.45;   // cap turret power
    private static final double DEADBAND_DEG   = 1.5;    // don't move when |yaw| < this
    private static final double SEARCH_POWER   = 0.10;   // slow scan when tag not found

    // target: aim directly at tag (yaw ~ 0)
    private static final int PREFERRED_TAG_ID  = -1;     // -1 = any tag; or set to 0..20 for one tag

    // Soft limits (encoder ticks) to protect wires/mechanics
    // Adjust these to match your turret's safe range.
    private static final int SOFT_MIN_TICKS    = -2600;
    private static final int SOFT_MAX_TICKS    =  2600;
    // --------------------------------------------------

    private DcMotor turret;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    // simple PID state
    private double lastErrorDeg = 0.0;
    private double errorIntegral = 0.0;
    private double lastTime = 0.0;

    @Override
    public void runOpMode() {
        // --------------- Hardware ----------------
        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        // If turret spins the wrong direction, flip this:
        // turret.setDirection(DcMotorSimple.Direction.REVERSE);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);

        // Reset encoder at startup
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // --------------- Vision ------------------
        aprilTag = new AprilTagProcessor.Builder().build();

        WebcamName webcam = hardwareMap.get(WebcamName.class, "webcam");
        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Turret AprilTag Aimer: INIT");
        telemetry.addLine("A: stop stream  |  B: resume stream  |  X: re-zero encoder");
        telemetry.update();

        waitForStart();

        lastTime = getRuntime();

        while (opModeIsActive()) {
            // --- Controls for testing ---
            if (gamepad1.a) {
                visionPortal.stopStreaming();
            }
            if (gamepad1.b) {
                visionPortal.resumeStreaming();
            }
            if (gamepad1.x) { // manual zero where you are
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            List<AprilTagDetection> detections = aprilTag.getDetections();
            double commandedPower;

            if (detections != null && !detections.isEmpty()) {
                AprilTagDetection d = pickDetection(detections, PREFERRED_TAG_ID);

                // In FTC, ftcPose.yaw is in degrees.
                // Target is yaw -> 0 (tag centered).
                double yawDeg = d.ftcPose.yaw;

                // Deadband to avoid twitching around perfect center
                if (Math.abs(yawDeg) < DEADBAND_DEG) {
                    commandedPower = 0.0;
                    resetPid();
                } else {
                    commandedPower = pidUpdate(yawDeg);
                    // add static feedforward to overcome stiction if commanding nonzero
                    commandedPower += KS * Math.signum(commandedPower);
                }

            } else {
                // No tag: do a slow periodic scan (sine sweep)
                commandedPower = SEARCH_POWER * Math.signum(Math.sin(getRuntime() * 0.5));
                resetPid();
            }

            // Enforce soft limits
            int pos = turret.getCurrentPosition();
            if (pos <= SOFT_MIN_TICKS && commandedPower < 0) {
                commandedPower = 0;
            }
            if (pos >= SOFT_MAX_TICKS && commandedPower > 0) {
                commandedPower = 0;
            }

            // clamp and apply
            commandedPower = clamp(commandedPower, -MAX_POWER, MAX_POWER);
            turret.setPower(commandedPower);

            // Telemetry
            telemetry.addData("turret ticks", pos);
            telemetry.addData("power", "%.2f", commandedPower);

            if (detections != null && !detections.isEmpty()) {
                AprilTagDetection use = pickDetection(detections, PREFERRED_TAG_ID);
                telemetry.addData("tag count", detections.size());
                telemetry.addData("using tag id", use.id);
                telemetry.addData("yaw deg", "%.2f", use.ftcPose.yaw);
            } else {
                telemetry.addLine("No tag detected — sweeping…");
            }

            telemetry.update();
        }

        // cleanup
        turret.setPower(0);
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

    // Pick a tag: preferred ID if available, otherwise the one closest to center.
    private AprilTagDetection pickDetection(List<AprilTagDetection> list, int preferredId) {
        if (preferredId >= 0) {
            for (AprilTagDetection d : list) {
                if (d.id == preferredId) return d;
            }
        }
        // otherwise, pick the one with minimal |yaw|
        AprilTagDetection best = list.get(0);
        double bestAbs = Math.abs(best.ftcPose.yaw);
        for (AprilTagDetection d : list) {
            double a = Math.abs(d.ftcPose.yaw);
            if (a < bestAbs) {
                best = d;
                bestAbs = a;
            }
        }
        return best;
    }

    // PID controller on yaw error (target = 0 deg)
    private double pidUpdate(double errorYawDeg) {
        double now = getRuntime();
        double dt = Math.max(1e-3, now - lastTime); // avoid divide-by-zero

        double derivative = (errorYawDeg - lastErrorDeg) / dt;
        errorIntegral += errorYawDeg * dt;

        double out = KP * errorYawDeg + KD * derivative + KI * errorIntegral;

        lastErrorDeg = errorYawDeg;
        lastTime = now;

        // Negative sign because if yaw is +, turret likely must turn - to center.
        // If it moves the wrong way, change this to `return +out;`
        return -out;
    }

    private void resetPid() {
        lastErrorDeg = 0.0;
        errorIntegral = 0.0;
        lastTime = getRuntime();
    }

    private double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}