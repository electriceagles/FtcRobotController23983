package org.firstinspires.ftc.teamcode.LimelightTest;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="AprilTag Localization Shooter", group="Vision")
public class DistanceCalculator extends LinearOpMode{
    public Limelight3A limelight;
    public double tx = 0;
    public double ty = 0;

    public DcMotorEx shooter1;
    public DcMotorEx shooter2;

    // degree of limelight mount
    public double mountAng = 6.7;

    //height relative to ground
    public double llheighIn = 20.0;

    //height of goal
    public double goalHeightInches = 60.0;



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

            double targetdeltaY = ty;


            double angleToGoalDegrees = mountAng + targetdeltaY;
            double angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);

            //calculate distance and use to regulate stuff
            double distanceLL2G = (goalHeightInches - llheighIn) / Math.tan(angleToGoalRadians);
        }


        }
    }
