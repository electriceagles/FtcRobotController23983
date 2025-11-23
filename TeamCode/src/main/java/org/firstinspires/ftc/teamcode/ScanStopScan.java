package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

@TeleOp(name = "Scan Stop Scan", group = "Vision")
public class ScanStopScan extends LinearOpMode {

    private static final String TURRET_MOTOR = "turret";
    private static final String WEBCAM_NAME  = "webcam";

    private static final double SCAN_POWER = 0.20;  // speed while scanning
    private static final double SCAN_FREQ  = 0.25;  // sweep cycles per second

    private DcMotor turret;
    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {

        // ----- MOTOR -----
        turret = hardwareMap.get(DcMotor.class, TURRET_MOTOR);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // ----- VISION -----
        aprilTag = new AprilTagProcessor.Builder().build();

        WebcamName cam = hardwareMap.get(WebcamName.class, WEBCAM_NAME);

        visionPortal = new VisionPortal.Builder()
                .setCamera(cam)
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .enableLiveView(true)
                .addProcessor(aprilTag)
                .build();

        telemetry.addLine("Scan → Stop when tag visible → Scan again when tag disappears.");
        telemetry.update();

        waitForStart();

        double t0 = getRuntime();

        while (opModeIsActive()) {

            List<AprilTagDetection> detections = aprilTag.getDetections();

            boolean tagVisible = (detections != null && !detections.isEmpty());

            if (tagVisible) {
                // ----- TAG VISIBLE → STOP -----
                turret.setPower(0);

                telemetry.addLine("TAG SEEN → STOP");
                telemetry.addData("Tag Count", detections.size());
                telemetry.update();

            } else {
                // ----- NO TAG VISIBLE → SCAN -----
                double t = getRuntime() - t0;
                double s = Math.sin(2 * Math.PI * SCAN_FREQ * t); // -1 to 1

                double power = SCAN_POWER * Math.signum(s);
                turret.setPower(power);

                telemetry.addLine("NO TAG → SCANNING…");
                telemetry.addData("Power", power);
                telemetry.update();
            }
        }

        // End
        turret.setPower(0);
        if (visionPortal != null) visionPortal.close();
    }
}
