package org.firstinspires.ftc.teamcode.basicOpMode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Q2 TeleOp - Draft", group = "TeleOp")
public class Test extends LinearOpMode {

    public DcMotorEx lf;
    public DcMotorEx lr;
    public DcMotorEx rf;
    public DcMotorEx rr;

    public double lastSeenTagTime;

    public DcMotorEx intake;
    public Servo servo;
    public DcMotorEx shooter1;
    public DcMotorEx shooter2;
    public DcMotor turret;

    public double powerMult = 0.7;
    public double powerLim = 0.8;

    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTag;

    public static final int at = 24; // 24 for red?
    public static final double SCAN_POWER = 0.65;
    public static final double SCAN_FREQ  = 0.25;

    public boolean turretAuto = true;
    public boolean manControl = false;
    public double scanStartTime;

    public double s_targetRPM = 4200; // tune later

    public double kP = 0.0005;
    public double kI = 0.0;
    public double kD = 0.00012;

    private double shooterIntegral = 0;
    private double shooterLastError = 0;

    private int lastShooterTicks = 0;
    private double lastShooterTime = 0;

    private static final double TICKS_PER_REV = 28.0;

    // Finds the target AprilTag by ID
    private AprilTagDetection getTargetTag(List<AprilTagDetection> detections) {
        if (detections == null) return null;

        for (AprilTagDetection tag : detections) {
            if (tag.id == at) {
                return tag;
            }
        }
        return null;
    }

    @Override
    public void runOpMode() {

        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lr = hardwareMap.get(DcMotorEx.class, "lr");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rr = hardwareMap.get(DcMotorEx.class, "rr");

        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lr.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "i");
        servo = hardwareMap.get(Servo.class, "servo");
        shooter1 = hardwareMap.get(DcMotorEx.class, "sf1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "sf2");

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        aprilTag = new AprilTagProcessor.Builder().build();
        WebcamName webcam = hardwareMap.get(WebcamName.class, "webcam");

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .addProcessor(aprilTag)
                .build();

        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        scanStartTime = getRuntime();
        lastSeenTagTime = getRuntime();

        lastShooterTime = getRuntime();
        lastShooterTicks = shooter1.getCurrentPosition();

        while (opModeIsActive()) {

            double y  = gamepad1.left_stick_y;
            double x  = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            if (Math.abs(y)  < 0.05) y  = 0;
            if (Math.abs(x)  < 0.05) x  = 0;
            if (Math.abs(rx) < 0.05) rx = 0;

            double lfP = y + x + rx;
            double rfP = y - x - rx;
            double lrP = y - x + rx;
            double rrP = y + x - rx;

            lf.setPower(lfP * powerMult);
            rf.setPower(rfP * powerMult);
            lr.setPower(lrP * powerMult);
            rr.setPower(rrP * powerMult);

            powerLim = gamepad2.circle ? 0.7 : 0.8;

            if (gamepad1.right_bumper) {
                double currentRPM = getShooterRPM();
                double pidPower = shooterPID(currentRPM);

                shooter1.setPower(pidPower);
                shooter2.setPower(pidPower);

                telemetry.addData("Target RPM", s_targetRPM);
                telemetry.addData("Current RPM", currentRPM);
                telemetry.addData("Shooter Power", pidPower);
                telemetry.update();
            } else {
                shooter1.setPower(0);
                shooter2.setPower(0);

                shooterIntegral = 0;
                shooterLastError = 0;

                lastShooterTime = getRuntime();
                lastShooterTicks = shooter1.getCurrentPosition();
            }

            if (gamepad1.left_trigger > 0.1) {
                intake.setPower(1);
            } else if (gamepad1.left_bumper) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            if (gamepad1.right_bumper) {
                servo.setPosition(0.67);
            } else {
                servo.setPosition(0);
            }

            if (gamepad1.cross) {
                turretAuto = false;
                manControl = true;
            } else if (gamepad1.triangle) {
                turretAuto = true;
                manControl = false;
            }

            if (manControl) {
                if (gamepad1.dpad_left) {
                    turret.setPower(-0.4);
                } else if (gamepad1.dpad_right) {
                    turret.setPower(0.4);
                } else {
                    turret.setPower(0);
                }
            }

            if (turretAuto) {
                List<AprilTagDetection> detections = aprilTag.getDetections();
                AprilTagDetection targetTag = getTargetTag(detections);

                if (targetTag != null) {
                    lastSeenTagTime = getRuntime();
                }

                if (getRuntime() - lastSeenTagTime < 0.3) {
                    turret.setPower(0);
                } else {
                    double t = getRuntime() - scanStartTime;
                    double s = Math.sin(2 * Math.PI * SCAN_FREQ * t);
                    turret.setPower(SCAN_POWER * Math.signum(s));
                }
            }
        }

        turret.setPower(0);
        if (visionPortal != null) visionPortal.close();
    }

    private double getShooterRPM() {
        int currentTicks = shooter1.getCurrentPosition();
        double currentTime = getRuntime();

        double deltaTicks = currentTicks - lastShooterTicks;
        double deltaTime  = currentTime - lastShooterTime;

        if (deltaTime <= 0) return 0;

        double revolutions = deltaTicks / TICKS_PER_REV;
        double rpm = (revolutions / deltaTime) * 60.0;

        lastShooterTicks = currentTicks;
        lastShooterTime = currentTime;

        return Math.abs(rpm);
    }

    private double shooterPID(double currentRPM) {
        double error = s_targetRPM - currentRPM;

        shooterIntegral += error;
        double derivative = error - shooterLastError;
        shooterLastError = error;

        double output = (kP * error) + (kI * shooterIntegral) + (kD * derivative);
        return Math.max(0.0, Math.min(1.0, output));
    }
}
