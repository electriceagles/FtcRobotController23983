package org.firstinspires.ftc.teamcode.superTele.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@Configurable
@TeleOp
public class ShowcaseTeleOp extends OpMode {

    DcMotorEx lf, lr, rf, rr;

    // Mechanisms
    DcMotorEx intake;
    DcMotorEx turret;
    DcMotorEx shooter1, shooter2;


    // Shooter tuning
    double P = 111.40100;
    double F = 14.05;
    double curTargetVelocity = 0;

    // servo


    // field target (in inches for red)


    public static double drivePower = 0.8;

    @Override
    public void init() {
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorEx.Direction.REVERSE);



        // drive
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lr = hardwareMap.get(DcMotorEx.class, "lr");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rr = hardwareMap.get(DcMotorEx.class, "rr");

        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lr.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        // intake
        intake = hardwareMap.get(DcMotorEx.class, "i");


        shooter1 = hardwareMap.get(DcMotorEx.class, "sf1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "sf2");

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

    }

    @Override
    public void loop(){
        //drive
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        lf.setPower((y + x + rx * 0.8) * drivePower);
        rf.setPower((y - x - rx * 0.8) * drivePower);
        lr.setPower((y - x + rx * 0.8) * drivePower);
        rr.setPower((y + x - rx * 0.8) * drivePower);

        // intake or transfer
        if (gamepad1.dpad_up) {
            intake.setPower(1);
        } else {
            intake.setPower(0);
        }


        if (gamepad1.left_trigger > 0.1) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        if (gamepad1.dpad_left) {
            turret.setPower(0.3);
        } else if (gamepad1.dpad_right) {
            turret.setPower(-0.3);
        } else {
            turret.setPower(0);
        }

        // shooting velocity selector
        if (gamepad1.triangle) {
            curTargetVelocity = 1040;
        } else if (gamepad1.square) {
            curTargetVelocity = 1120;
        } else if (gamepad1.cross) {
            curTargetVelocity = 1580;
        } else if (gamepad1.right_bumper) {
            curTargetVelocity = 1200;
        } else {
            curTargetVelocity = 0;
        }


        shooter1.setVelocity(curTargetVelocity);
        shooter2.setVelocity(curTargetVelocity);

        double curVelocity1 = shooter1.getVelocity();

        double error1 = curVelocity1 - curTargetVelocity;

        if (curTargetVelocity > 0 && Math.abs(error1) < 50){
            gamepad1.rumbleBlips(2);
        }
    }

}
