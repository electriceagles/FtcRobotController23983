package org.firstinspires.ftc.teamcode.basicOpMode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Duo TeleOp + Vision Turret", group = "TeleOp")
public class TeleOpFinal extends LinearOpMode {

    public DcMotorEx lf;
    public DcMotorEx lr;
    public DcMotorEx rf;
    public DcMotorEx rr;

    public DcMotorEx intake;
    public DcMotorEx shooter1;
    public DcMotorEx shooter2;
    public DcMotor turret;

    public double powerMult = 0.5;
    public double shooterLimit = 1.0;

    public double mp = 0.4;

    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTag;


    public static final double SCAN_POWER = 0.30;
    public static final double SCAN_FREQ  = 0.25;

    public boolean turretAuto = true;
    public double scanStartTime;

    @Override
    public void runOpMode() {


        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lr = hardwareMap.get(DcMotorEx.class, "lr");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rr = hardwareMap.get(DcMotorEx.class, "rr");

        //*note to self: change the directions later once robot is finished*
        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lr.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);


        intake = hardwareMap.get(DcMotorEx.class, "i");
        shooter1 = hardwareMap.get(DcMotorEx.class, "sf1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "sf2");

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


        waitForStart();
        scanStartTime = getRuntime();

        while (opModeIsActive()) {

            // controller 1 drives the bot
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            double lfP = y + x + rx;
            double rfP = y - x - rx;
            double lrP = y - x + rx;
            double rrP = y + x - rx;


            lf.setPower(lfP * powerMult);
            rf.setPower(rfP * powerMult);
            lr.setPower(lrP * powerMult);
            rr.setPower(rrP * powerMult);

            // controller 1 intake
            if (gamepad1.dpad_up) {
                intake.setPower(-1); //artifact goes in w left trigger
            } else if (gamepad1.dpad_down) {
                intake.setPower(1); //artifact goes out w right trigger
            } else {
                intake.setPower(0);
            }

            // controller 2 shoots
            shooterLimit = gamepad2.b ? 0.6 : 1.0;

            if (gamepad2.right_trigger > 0.05) {
                double p = gamepad2.right_trigger * shooterLimit;
                shooter1.setPower(p);
                shooter2.setPower(p);
            } else {
                shooter1.setPower(0);
                shooter2.setPower(0);
            }

            // code for turret

            //manual turret control source
            double manualTurret = gamepad2.right_stick_x;

            if (Math.abs(manualTurret) > 0.1) {
                // overriding the auto scan
                turretAuto = false; //if the right stick moved, ignores auto scan
                double m = manualTurret * mp; //limits the manual turret power to 40% of trigger movement
                turret.setPower(m);
            } else {
                turretAuto = true;
            }
            //note for driver: only move to manual if auto scan doesn't work

            //automatic turret scan system (uses sin wave's concept to scan left and right)
            if (turretAuto) {
                List<AprilTagDetection> detections = aprilTag.getDetections();
                boolean tagSeen = detections != null && !detections.isEmpty();

                if (tagSeen) {
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
}