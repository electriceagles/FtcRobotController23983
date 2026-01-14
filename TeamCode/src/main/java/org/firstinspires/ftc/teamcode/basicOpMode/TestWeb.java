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

@TeleOp(name = "Q2 TeleOp + Webcam", group = "TeleOp")
public class TestWeb extends LinearOpMode {

    public DcMotorEx lf;
    public DcMotorEx lr;
    public DcMotorEx rf;
    public DcMotorEx rr;

    public DcMotorEx turret;

    public DcMotorEx intake;
    public Servo servo;
    public DcMotorEx shooter1;
    public DcMotorEx shooter2;

    public double powerMult = 0.9;

    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTag;
    public static final int at = 24; // 24 for red?

    public static double SCAN_POWER = 0;
    public static final double SCAN_FREQ  = 0.25;

    public boolean turretAuto = false;
    public boolean manControl = true;
    public double scanStartTime;

    public double lastSeenTagTime = 0;


    public double s_targetRPM = 0; // tune later

    public double kP = 0.0005;
    public double kI = 0.0;
    public double kD = 0.00012;

    private double shooterIntegral = 0;
    private double shooterLastError = 0;

    private double lastShooterTicks = 0;
    private double lastShooterTime = 0;

    private static final double TICKS_PER_REV = 28.0;

    // basically if it detects a tag, it checks if it's the one we need
    private AprilTagDetection getTargetTag(List<AprilTagDetection> detections) { //lists all detections and checks, this the target tag function we use later
        if (detections == null) return null;

        for (AprilTagDetection tag : detections) { //checks each; for loop
            if (tag.id == at) {
                return tag; // stops loop
            }
        }
        return null; //only if detection didnt detect nun return null
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

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        aprilTag = new AprilTagProcessor.Builder().build();
        WebcamName webcam = hardwareMap.get(WebcamName.class, "webcam");

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .addProcessor(aprilTag)
                .build();

        shooter1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooter2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();

        lastShooterTime = getRuntime();
        lastShooterTicks = shooter1.getCurrentPosition();

        scanStartTime = getRuntime();
        lastSeenTagTime = getRuntime();

        while (opModeIsActive()) {

            // Drive
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


            //change rpm values:
            if (gamepad1.triangle){
                s_targetRPM = 3000;
            } else if (gamepad1.square){
                s_targetRPM = 4000;
            } else if (gamepad1.cross) {
                s_targetRPM = 5000;
            } else if (gamepad1.right_bumper) {
                s_targetRPM = 6000;
            } else {
                s_targetRPM = 0;
            }

            // Shooter
            if (s_targetRPM > 0) {
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
                lastShooterTicks = (shooter1.getCurrentPosition() + shooter2.getCurrentPosition()) / 2.0;
            }

            // Intake
            if (gamepad1.left_trigger > 0.1) {
                intake.setPower(1);
            } else if (gamepad1.left_bumper) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            if (gamepad1.dpadUpWasPressed()) {
                SCAN_POWER = 0.8;
                turretAuto = true;
                manControl = false;
            } else if (gamepad1.dpadDownWasPressed()) {
                SCAN_POWER = -0.8;
                turretAuto = true;
                manControl = false;
            } else if (gamepad1.rightBumperWasReleased() || gamepad1.triangleWasReleased() || gamepad1.squareWasReleased() || gamepad1.crossWasReleased()) {
                manControl = true;
                turretAuto = false;
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
        double currentTicks = (shooter1.getCurrentPosition() + shooter2.getCurrentPosition()) / 2.0;
        double currentTime = getRuntime();

        double deltaTicks = currentTicks - lastShooterTicks;
        double deltaTime = currentTime - lastShooterTime;

        if (deltaTime <= 0) return 0;

        double revolutions = deltaTicks / TICKS_PER_REV;
        double rpm = (revolutions / deltaTime) * 60.0;

        lastShooterTicks = currentTicks;
        lastShooterTime = currentTime;

        return Math.abs(rpm);
    }

    //also:
    //P term: pushes harder the farther we are from target rpm
    //D term: reacts to the change in error (damps overshoot / oscillation)

    //actually applying pid for shooter
    private double shooterPID(double currentRPM) {

        //our error
        double error = s_targetRPM - currentRPM;


        //accumulation of error over time (net change)
        shooterIntegral += error;

        //looks at how quickly the error is changing and helps brake so its not overpowered
        double derivative = error - shooterLastError;

        //save error for next calculation
        shooterLastError = error;

        // Combine PID terms into one value; formula: output = kP + kI + kD
        double output = (kP * error) + (kI * shooterIntegral) + (kD * derivative);

        //max motor power is 1
        output = Math.max(0.0, Math.min(1.0, output));

        return output; //makes shooterPID the value after getting all those numbers tg
    }
}
