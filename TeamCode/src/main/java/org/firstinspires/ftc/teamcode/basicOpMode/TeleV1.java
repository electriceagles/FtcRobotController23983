package org.firstinspires.ftc.teamcode.basicOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Q2 TeleOp (v1)", group = "TeleOp")
public class TeleV1 extends LinearOpMode {

    // Drive motors
    public DcMotorEx lf, lr, rf, rr;

    // Mechanisms
    public DcMotorEx intake;

    public DcMotorEx turret;
    public DcMotorEx shooter1, shooter2;

    public double powerMult = 0.9;

    public double power = 0;

    @Override
    public void runOpMode() {

        // Drive motors
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lr = hardwareMap.get(DcMotorEx.class, "lr");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rr = hardwareMap.get(DcMotorEx.class, "rr");

        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lr.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        // Intake & shooter
        intake = hardwareMap.get(DcMotorEx.class, "i");
        shooter1 = hardwareMap.get(DcMotorEx.class, "sf1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "sf2");

        turret = hardwareMap.get(DcMotorEx.class, "turret");

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);


        waitForStart();


        while (opModeIsActive()) {

            // ===== DRIVE =====
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

            // ===== SHOOTER SPEED =====
            double b = gamepad1.right_trigger;
            if (gamepad1.triangle) {
                power = 0.42;
            } else if (gamepad1.square) {
                power = 0.584;
            } else if (gamepad1.cross) {
                power = 0.7;
            } else if (gamepad1.right_bumper) {
                power = 0.45;
            } else if (gamepad1.right_trigger > 0.1) {
                power = -b;
            } else {
                power = 0;
            }

            shooter1.setPower(power);
            shooter2.setPower(power);

            if (gamepad1.dpad_right){
                turret.setPower(-0.4);
            } else if (gamepad1.dpad_left) {
                turret.setPower(0.4);
            } else {
                turret.setPower(0);
            }

            // Shooter PID


            // ===== INTAKE =====
            if (gamepad1.left_trigger > 0.1) {
                intake.setPower(-1);
            } else if (gamepad1.left_bumper) {
                intake.setPower(1);
            } else {
                intake.setPower(0);
            }
        }
    }
}