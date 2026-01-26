package org.firstinspires.ftc.teamcode.superTele;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class FlyWheelPIDFtuning extends OpMode {

    public DcMotorEx shooter1;
    public DcMotorEx shooter2;

    public double lowVelocity = 3000;
    public double medVelocity = 4000;
    public double interVelocity = 5000;
    public double highVelocity = 5500;

    double curTargetVelocity= 0;

    double P = 0;
    double F = 0;

    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.001};
    int stepIndex = 1;


    @Override
    public void init(){
        shooter1 = hardwareMap.get(DcMotorEx.class, "sf1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "sf2");

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);

        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        telemetry.addLine("Initial Setup done");
    }

    @Override
    public void loop(){
        // get gamepad command
        //set targ velo
        //update telemetry

     if (gamepad1.triangleWasPressed()){
         curTargetVelocity = lowVelocity;
     } else if (gamepad1.squareWasPressed()){
         curTargetVelocity = medVelocity;
     } else if (gamepad1.rightBumperWasPressed()) {
         curTargetVelocity = interVelocity;
     } else if (gamepad1.crossWasPressed()) {
         curTargetVelocity = highVelocity;
     } else {
         curTargetVelocity = 0;
     }

     if (gamepad1.circleWasPressed()) {
         stepIndex = (stepIndex + 1) % stepSizes.length;
     }

     if (gamepad1.dpadLeftWasPressed()){
         F -= stepSizes[stepIndex];
     }

     if (gamepad1.dpadRightWasPressed()) {
         F += stepSizes[stepIndex];
     }

     if (gamepad1.dpadUpWasPressed()) {
         P += stepSizes[stepIndex];
     }

     if (gamepad1.dpadDownWasPressed()) {
         P -= stepSizes[stepIndex];
     }

     //set new PIDF coef

     PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);

     shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
     shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

     //set new velocity

     shooter1.setVelocity(curTargetVelocity);
     shooter2.setVelocity(curTargetVelocity);

     double curVelocity1 = shooter1.getVelocity();
     double curVelocity2 = shooter2.getVelocity();

     double error1 = curVelocity1 - curTargetVelocity;
     double error2 = curVelocity2 - curTargetVelocity;

     //telemetry stuff

     telemetry.addData("Target Velocity", curTargetVelocity);
     telemetry.addData("Current Velocity Motor1", "%.2f", curVelocity1);
     telemetry.addData("Current Velocity Motor2", "%.2f", curVelocity2);
     telemetry.addData("Error Motor1", "%.2f", error1);
     telemetry.addData("Error Motor2", "%.2f", error2);
     telemetry.addLine("..........................................");
     telemetry.addLine("------------------------------------------");
     telemetry.addData("Tuning P", "%.4f (D-Pad U/D)", P);
     telemetry.addData("Tuning F", "%.4f (D-Pad L/F)", P);
    }
}
