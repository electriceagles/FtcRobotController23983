package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class TurretTagAimerModule {

    private final DcMotor turret;
    private final VisionPortal visionPortal;
    private final AprilTagProcessor aprilTag;

    private final double KP = 0.025;
    private final double KD = 0.002;
    private final double KS = 0.05;
    private final double MAX_POWER = 0.45;
    private final double DEADBAND_DEG = 1.5;

    private double lastError = 0;
    private double lastTime = 0;

    public TurretTagAimerModule(HardwareMap hw, double runtimeStart) {

        turret = hw.dcMotor.get("turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        aprilTag = new AprilTagProcessor.Builder().build();
        WebcamName webcam = hw.get(WebcamName.class, "webcam");

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .addProcessor(aprilTag)
                .build();

        lastTime = runtimeStart;
    }

    public void update(double runtime) {

        List<AprilTagDetection> detections = aprilTag.getDetections();

        double power;

        if (detections != null && !detections.isEmpty()) {
            AprilTagDetection d = detections.get(0);
            double yaw = d.ftcPose.yaw;

            if (Math.abs(yaw) < DEADBAND_DEG) {
                power = 0;
                resetPID(runtime);
            } else {
                double dt = Math.max(1e-3, runtime - lastTime);
                double derivative = (yaw - lastError) / dt;

                power = -(KP * yaw + KD * derivative);
                power += KS * Math.signum(power);
            }
        } else {
            // no tag → slow scan
            power = 0.1 * Math.signum(Math.sin(runtime * 0.5));
            resetPID(runtime);
        }

        // clamp
        power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));
        turret.setPower(power);

        lastTime = runtime;
    }

    private void resetPID(double time) {
        lastError = 0;
        lastTime = time;
    }

    public void stop() {
        turret.setPower(0);
        visionPortal.close();
    }
}
