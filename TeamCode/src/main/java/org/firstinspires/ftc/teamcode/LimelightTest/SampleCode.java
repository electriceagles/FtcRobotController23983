package org.firstinspires.ftc.teamcode.LimelightTest;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;


@TeleOp(name="AprilTag Test Limelight", group="Vision")
public class SampleCode extends LinearOpMode {
    public Limelight3A limelight;
    public double tx = 0;
    public double ty = 0;

    public DcMotorEx turret;

    @Override
    public void runOpMode() {
        limelight = hardwareMap.get(Limelight3A.class, "Limelight");
        limelight.pipelineSwitch(2); //2 for now but depends on which one we use

        waitForStart();

        limelight.start(); //startup

        while (opModeIsActive()) {
            LLStatus status = limelight.getStatus(); //info abt limelight, helpful for debugging and stuff
            telemetry.addData("Name", "%s", status.getName());

            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d", status.getTemp(), status.getCpu(), (int) status.getFps());

            telemetry.addData("Pipeline", "Index: %d, Type: %s", status.getPipelineIndex(), status.getPipelineType());

            //telemetry stuff is from demonsource

            LLResult result = limelight.getLatestResult(); // for result we can make limelight have any other detections other than our target count as null thru id filtering on frontend

            if (result != null && result.isValid()) {
                tx = result.getTx(); // How far left or right the target is (degrees)
                ty = result.getTy(); // How far up or down the target is (degrees)

                telemetry.addData("Target distance X:", tx);
                telemetry.addData("Target distance Y:", ty);
            } else {
                telemetry.addData("Limelight:", "No Targets");
            }

            if (tx > 5) { //basically driver needs to align/turn robot until the tag is at least within 5 degrees of view
                turret.setPower(0.4); //might need to change based on how fast limelight processing is
            } else if (tx < -5) {
                turret.setPower(-0.4);
            } else {
                turret.setPower(0);
            }
        }

    }
}