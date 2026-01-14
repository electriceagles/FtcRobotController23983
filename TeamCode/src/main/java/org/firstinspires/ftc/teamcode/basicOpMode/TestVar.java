package org.firstinspires.ftc.teamcode.basicOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Q2 TeleOp (No Turret)", group = "TeleOp")
public class TestVar extends LinearOpMode {

    // Drive motors
    public DcMotorEx lf, lr, rf, rr;

    // Mechanisms
    public DcMotorEx intake;
    public DcMotorEx shooter1, shooter2;
    public Servo servo;

    public double powerMult = 0.9;

    // Shooter PID
    public double power = 0;

    private static final double TICKS_PER_REV = 28.0;

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
        servo = hardwareMap.get(Servo.class, "servo");
        shooter1 = hardwareMap.get(DcMotorEx.class, "sf1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "sf2");

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        waitForStart();


        while (opModeIsActive()) {

            // ===== DRIVE =====
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            if (Math.abs(y) < 0.05) y = 0;
            if (Math.abs(x) < 0.05) x = 0;
            if (Math.abs(rx) < 0.05) rx = 0;

            lf.setPower((y + x + rx) * powerMult);
            rf.setPower((y - x - rx) * powerMult);
            lr.setPower((y - x + rx) * powerMult);
            rr.setPower((y + x - rx) * powerMult);

            // ===== SHOOTER SPEED =====
            if (gamepad1.triangle) {
                power = 0.5;
            } else if (gamepad1.square) {
                power = 0.67;
            } else if (gamepad1.cross) {
                power = 0.83;
            } else if (gamepad1.right_bumper) {
                power = 1;
            } else {
                power = 0;
            }

            // Shooter PID
            shooter1.setPower(power);
            shooter2.setPower(power);

            if (gamepad1.right_trigger > 0.1){
                shooter1.setPower(-1);
                shooter2.setPower(-1);
            }

            // ===== INTAKE =====
            if (gamepad1.left_trigger > 0.1) {
                intake.setPower(1);
            } else if (gamepad1.left_bumper) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }
        }
    }
}