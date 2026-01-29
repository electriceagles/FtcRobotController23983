package org.firstinspires.ftc.teamcode.LimelightTest;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="AprilTag Localization Shooter", group="Vision")
public class DistanceShooter extends LinearOpMode{
    public Limelight3A limelight;
    public double tx = 0;
    public double ty = 0;

    public DcMotorEx shooter1;
    public DcMotorEx shooter2;



    @Override
    public void runOpMode(){
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(2); //2 for now but depends on which one we use

        waitForStart();

        limelight.start(); //startup

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {
                tx = result.getTx(); // How far left or right the target is (degrees)
                ty = result.getTy(); // How far up or down the target is (degrees)

                telemetry.addData("Target angle X:", tx);
                telemetry.addData("Target angle Y:", ty);
            } else {
                telemetry.addData("AprilTag:", "No Targets");
            }

            double targetOffsetAngle_Vertical = ty;


            // how many degrees back is your limelight rotated from perfectly vertical?
            double limelightMountAngleDegrees = 25.0;

            // distance from the center of the Limelight lens to the floor
            double limelightLensHeightInches = 20.0;

            // distance from the target to the floor
            double goalHeightInches = 60.0;

            double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
            double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

            //calculate distance
            double distanceLL2G = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
        }


        }
    }
