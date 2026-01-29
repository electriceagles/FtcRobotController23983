package org.firstinspires.ftc.teamcode.Auton.Logics;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
public class LimeDistance {
    Limelight3A limelight;

    public HardwareMap hardwareMap;
    public Servo servo;
    // Constants
    double limelightMountAngleDegrees = 25.0; // Example
    double limelightLensHeightInches = 20.0;
    double goalHeightInches = 60.0;

    // Get target vertical angle (ty) from Limelight
    LLResult result = limelight.getLatestResult();
        double ty = result.getTy(); // How far up or down the target is (degrees)

    // Calculate distance
    double angleToGoalDegrees = limelightMountAngleDegrees + ty;
    double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
    double distance = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);


    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start(); // This tells Limelight to start looking!
    }

    public double getDistance() {
        return distance;
    }
}
