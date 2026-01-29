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
    double limelightMountAngleDegrees = 25.0; // Example
    double limelightLensHeightInches = 20.0;
    double goalHeightInches = 60.0;
    public Double distance;

    public void update(){
    // Get target vertical angle (ty) from Limelight
    LLResult result = limelight.getLatestResult();
        double ty = result.getTy(); // How far up or down the target is (degrees)

    // Calculate distance
    double angleToGoalDegrees = limelightMountAngleDegrees + ty;
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
    distance = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
}

    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
    }


}
