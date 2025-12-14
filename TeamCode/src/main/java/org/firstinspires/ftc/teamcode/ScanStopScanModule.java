package org.firstinspires.ftc.teamcode;

import android.util.Size;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class ScanStopScanModule {

    private final DcMotor turret;
    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTag;
    private final double SCAN_FREQ = 0.25;
    private final double SCAN_POWER = 0.20;

    private double startTime;

    public ScanStopScanModule(HardwareMap hw, double runtimeStart) {
        startTime = runtimeStart;

        turret = hw.dcMotor.get("turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        aprilTag = new AprilTagProcessor.Builder().build();
        WebcamName cam = hw.get(WebcamName.class, "webcam");

        visionPortal = new VisionPortal.Builder()
                .setCamera(cam)
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag)
                .build();
    }

    public void update(double runtime) {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        boolean tagVisible = detections != null && !detections.isEmpty();

        if (tagVisible) {
            turret.setPower(0);
        } else {
            double t = runtime - startTime;
            double s = Math.sin(2 * Math.PI * SCAN_FREQ * t);
            double power = SCAN_POWER * Math.signum(s);
            turret.setPower(power);
        }
    }

    public void stop() {
        turret.setPower(0);
        visionPortal.close();
    }
}
