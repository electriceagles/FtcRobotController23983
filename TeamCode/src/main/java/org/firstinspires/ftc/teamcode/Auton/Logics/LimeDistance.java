package org.firstinspires.ftc.teamcode.Auton.Logics;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
public class LimeDistance {
    public HardwareMap hardwareMap;
    private Limelight3A limelight;

    public Servo servo;
    // Constants
    double limelightMountAngleDegrees = 0; // Example
    double limelightLensHeightInches = 13.4535433;
    double goalHeightInches = 29.67;
    public Double distance;

    public void update(){
    // Get target vertical angle (ty) from Limelight
    LLResult result = limelight.getLatestResult();
        double ty = result.getTy(); // How far up or down the target is (degrees)

    // Calculate distance
    double angleToGoalDegrees = limelightMountAngleDegrees + ty;
    double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);
    distance = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
}

    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.setPollRateHz(100);
        limelight.start();
    }


}
