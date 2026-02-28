package org.firstinspires.ftc.teamcode.Auton.Logics;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class TurretLogic {
    private Limelight3A limelight;
    public double tx = 0;

    public DcMotorEx turret;

    public void init(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        limelight.pipelineSwitch(1);

        limelight.start(); //startup
        LLStatus status = limelight.getStatus(); //info abt limelight, helpful for debugging and stuff
        telemetry.addData("Name", "%s", status.getName());

        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d", status.getTemp(), status.getCpu(), (int) status.getFps());

        telemetry.addData("Pipeline", "Index: %d, Type: %s", status.getPipelineIndex(), status.getPipelineType());


    }
    public void update() {
        LLResult result = limelight.getLatestResult(); // for result we can make limelight have any other detections other than our target count as null thru id filtering on frontend
        if (result != null && result.isValid()) {
            tx = result.getTx();
            // Simple Proportional-ish control
            if (tx > 2) {
                turret.setPower(0.3);
            } else if (tx < -2) {
                turret.setPower(-0.3);
            } else if(tx < 2 && tx > -2){
                turret.setPower(0);
            }
        } else {
            turret.setPower(0); // Stop if target lost
        }
    }
}