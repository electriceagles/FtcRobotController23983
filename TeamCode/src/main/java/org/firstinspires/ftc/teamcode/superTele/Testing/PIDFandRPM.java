package org.firstinspires.ftc.teamcode.superTele.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp
public class PIDFandRPM extends OpMode {

    public DcMotorEx lf, lr, rf, rr;

    public DcMotorEx intake;
    public double powerMult = 0.7;

    double curTargetVelocity= 0;

    double P = 0;
    double F = 0;


    public DcMotorEx shooter1;
    public DcMotorEx shooter2;


    double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.001};
    int stepIndex = 1;

    double[] RPMstep = {1000.0, 100.0, 10.0, 5.0};
    int RPMindex = 0;


    @Override
    public void init(){
        shooter1 = hardwareMap.get(DcMotorEx.class, "sf1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "sf2");

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lr = hardwareMap.get(DcMotorEx.class, "lr");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rr = hardwareMap.get(DcMotorEx.class, "rr");

        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lr.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "i");

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

        //drive
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        if (Math.abs(y) < 0.05) y = 0;
        if (Math.abs(x) < 0.05) x = 0;
        if (Math.abs(rx) < 0.05) rx = 0;

        lf.setPower((y + x + rx * 0.8) * powerMult);
        rf.setPower((y - x - rx * 0.8) * powerMult);
        lr.setPower((y - x + rx * 0.8) * powerMult);
        rr.setPower((y + x - rx * 0.8) * powerMult);

        //intake and transfer
        if (gamepad1.left_trigger > 0.1) {
            intake.setPower(-1);
        } else if (gamepad1.right_trigger > 0.1) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }


        //set and or change RPM
        if (gamepad1.squareWasPressed()) {
            RPMindex = (RPMindex + 1) % RPMstep.length;
        }

        if (gamepad1.triangleWasPressed()){
            curTargetVelocity += RPMstep[RPMindex];
        }

        if (gamepad1.crossWasPressed()){
            curTargetVelocity -= RPMstep[RPMindex];
        }

        //tune P and F
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
        if (gamepad1.right_bumper){
            shooter1.setVelocity(curTargetVelocity);
            shooter2.setVelocity(curTargetVelocity);
        } else if (gamepad1.left_bumper) {
            shooter1.setVelocity(0);
            shooter2.setVelocity(0);
        }


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
        telemetry.addData("Tuning F", "%.4f (D-Pad L/F)", F);
        telemetry.addData("step size for P and F", "%.4f (B button)", stepSizes[stepIndex]);
        telemetry.addData("Velocity step size", "%.4f (Triangle and X)", RPMstep[RPMindex]);
        telemetry.addData("velocity,F pair", "%.1f , %.4f", curTargetVelocity, F);
    }
}
