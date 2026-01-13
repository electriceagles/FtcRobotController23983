package org.firstinspires.ftc.teamcode.basicOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Q2 TeleOp", group = "TeleOp")
public class TestEvt extends LinearOpMode {

    public DcMotorEx lf;
    public DcMotorEx lr;
    public DcMotorEx rf;
    public DcMotorEx rr;

    public DcMotorEx intake;
    public Servo servo;
    public DcMotorEx shooter1;
    public DcMotorEx shooter2;

    public double powerMult = 0.7;
    public double powerLim = 0.8;

    public double s_targetRPM = 4200; // tune later

    public double kP = 0.0005;
    public double kI = 0.0;
    public double kD = 0.00012;

    private double shooterIntegral = 0;
    private double shooterLastError = 0;

    private int lastShooterTicks = 0;
    private double lastShooterTime = 0;

    private static final double TICKS_PER_REV = 28.0;

    @Override
    public void runOpMode() {

        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lr = hardwareMap.get(DcMotorEx.class, "lr");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rr = hardwareMap.get(DcMotorEx.class, "rr");

        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lr.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "i");
        servo = hardwareMap.get(Servo.class, "servo");
        shooter1 = hardwareMap.get(DcMotorEx.class, "sf1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "sf2");

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        shooter2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        lastShooterTime = getRuntime();
        lastShooterTicks = shooter1.getCurrentPosition();

        while (opModeIsActive()) {

            // Drive
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

            powerLim = gamepad2.circle ? 0.7 : 0.8;

            // Shooter
            if (gamepad1.right_bumper) {
                double currentRPM = getShooterRPM();
                double pidPower = shooterPID(currentRPM);

                shooter1.setPower(pidPower);
                shooter2.setPower(pidPower);

                telemetry.addData("Target RPM", s_targetRPM);
                telemetry.addData("Current RPM", currentRPM);
                telemetry.addData("Shooter Power", pidPower);
                telemetry.update();
            } else {
                shooter1.setPower(0);
                shooter2.setPower(0);

                shooterIntegral = 0;
                shooterLastError = 0;

                lastShooterTime = getRuntime();
                lastShooterTicks = shooter1.getCurrentPosition();
            }

            // Intake
            if (gamepad1.left_trigger > 0.1) {
                intake.setPower(1);
            } else if (gamepad1.left_bumper) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            // Servo
            if (gamepad1.right_bumper) {
                servo.setPosition(0.67);
            } else {
                servo.setPosition(0);
            }
        }
    }

    private double getShooterRPM() {
        int currentTicks = shooter1.getCurrentPosition();
        double currentTime = getRuntime();

        double deltaTicks = currentTicks - lastShooterTicks;
        double deltaTime = currentTime - lastShooterTime;

        if (deltaTime <= 0) return 0;

        double revolutions = deltaTicks / TICKS_PER_REV;
        double rpm = (revolutions / deltaTime) * 60.0;

        lastShooterTicks = currentTicks;
        lastShooterTime = currentTime;

        return Math.abs(rpm);
    }

    private double shooterPID(double currentRPM) {
        double error = s_targetRPM - currentRPM;

        shooterIntegral += error;
        double derivative = error - shooterLastError;
        shooterLastError = error;

        double output = (kP * error) + (kI * shooterIntegral) + (kD * derivative);
        return Math.max(0.0, Math.min(1.0, output));
    }
}