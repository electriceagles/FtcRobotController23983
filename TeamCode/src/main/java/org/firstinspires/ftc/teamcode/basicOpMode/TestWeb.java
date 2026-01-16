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
    public ElapsedTime timer = new ElapsedTime();

    public boolean turretAuto = false;
    public boolean manControl = true;


    public double power = 0; // tune later




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

        visionPortal.setProcessorEnabled(aprilTag, false); //april tag processing off by default so memory doesnt get eaten

        shooter1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooter2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();

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
            if (gamepad1.triangle) {
                power = 0.1;
            } else if (gamepad1.square) {
                power = 0.2;
            } else if (gamepad1.cross) {
                power = 0.34;
            } else if (gamepad1.right_bumper) {
                power = 1;
            } else {
                power = 0;
            }

            shooter1.setPower(power);
            shooter2.setPower(power);


            // Intake
            if (gamepad1.left_trigger > 0.1) {
                intake.setPower(1);
            } else if (gamepad1.left_bumper) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            if (gamepad1.dpadUpWasPressed()) {
                timer.reset();
                visionPortal.setProcessorEnabled(aprilTag, true); //turns on processing when dpad is pressed
                SCAN_POWER = -0.8;
                turretAuto = true;
                manControl = false;
            } else if (gamepad1.dpadDownWasPressed()) {
                timer.reset();
                visionPortal.setProcessorEnabled(aprilTag, true);
                SCAN_POWER = 0.8;
                turretAuto = true;
                manControl = false;
            } else if (gamepad1.rightBumperWasReleased()
                    || gamepad1.triangleWasReleased()
                    || gamepad1.squareWasReleased()
                    || gamepad1.crossWasReleased()) {
                manControl = true;
                turretAuto = false;
                visionPortal.setProcessorEnabled(aprilTag, false); //turns of processing when shooter is off
            }


            if (manControl) {
                if (gamepad1.dpad_left){
                    turret.setPower(0.8);
                } else if (gamepad1.dpad_right) {
                    turret.setPower(-0.8);
                } else {
                    turret.setPower(0);
                }
            }


            if (turretAuto) {

                List<AprilTagDetection> detections = aprilTag.getDetections();
                AprilTagDetection targetTag = getTargetTag(detections);

                if (targetTag != null) {
                    turret.setPower(0);
                    turretAuto = false;
                    manControl = true;
                    visionPortal.setProcessorEnabled(aprilTag, false);
                }
                else if (timer.seconds() > 2.0) {
                    turret.setPower(0);
                    turretAuto = false;
                    manControl = true;
                    visionPortal.setProcessorEnabled(aprilTag, false);
                }
                else {
                    manControl = true;
                }
            } else {
                turret.setPower(0);
            }
        }
    }
}

