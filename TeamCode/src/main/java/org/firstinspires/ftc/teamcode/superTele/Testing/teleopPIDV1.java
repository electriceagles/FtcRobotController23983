package org.firstinspires.ftc.teamcode.superTele.Testing;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@TeleOp (name = "TeleOP + PID v1", group = "TeleOp")
public class teleopPIDV1 extends OpMode {

    public DcMotorEx lf, lr, rf, rr;

    public DcMotorEx intake;
    public double powerMult = 0.9;

    double curTargetVelocity= 0;

    double P = 111.40100;
    double F = 14.05;


    public DcMotorEx shooter1;
    public DcMotorEx shooter2;


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
        if (gamepad1.triangle) {
            curTargetVelocity = 1140;
        } else if (gamepad1.square) {
            curTargetVelocity = 1220;
        } else if (gamepad1.cross) {
            curTargetVelocity = 1280;
        } else if (gamepad1.right_bumper) {
            curTargetVelocity = 1360;
        } else {
            curTargetVelocity = 0;
        }

        //set new PIDF coef

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);

        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        //set new velocity
        shooter1.setVelocity(curTargetVelocity);
        shooter2.setVelocity(curTargetVelocity);


        //telemetry stuff

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("P", "%.4f", P);
        telemetry.addData("F", "%.4f", F);
    }
}
