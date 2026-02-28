package org.firstinspires.ftc.teamcode.superTele.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
@TeleOp(name = "Limelight Turret TeleOp", group = "TeleOp")
public class TeleopOdoV1 extends OpMode {

    // Drive motors
    DcMotorEx lf, lr, rf, rr;

    // Mechanisms
    DcMotorEx intake;
    DcMotorEx turret;
    DcMotorEx shooter1, shooter2;

    private Limelight3A limelight;

    public double tx = 0;
    public double ty = 0;

    private double lastError = 0;
    private static double kP = 0.1567;
    private static double kD = 0.08;

    private int toggle = 0;

    private ElapsedTime timer = new ElapsedTime();

    // Odometry
    GoBildaPinpointDriver pinpoint;

    // Shooter tuning
    double P = 111.40100;
    double F = 14.05;
    double curTargetVelocity = 0;

    double powerMult = 0.9;

    @Override
    public void init() {

        // Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);

        // Drive
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lr = hardwareMap.get(DcMotorEx.class, "lr");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rr = hardwareMap.get(DcMotorEx.class, "rr");

        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lr.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        // Intake
        intake = hardwareMap.get(DcMotorEx.class, "i");

        // Turret
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Shooter
        shooter1 = hardwareMap.get(DcMotorEx.class, "sf1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "sf2");

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        // Odometry
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        pinpoint.setEncoderResolution(
                GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(-84, -168, DistanceUnit.MM);

        pinpoint.setPosition(
                new Pose2D(DistanceUnit.INCH, 56, 8, AngleUnit.DEGREES, 90));

        telemetry.addLine("Starting Pose: 56, 8, 90º");
    }

    @Override
    public void start() {
        limelight.start();
        timer.reset();
    }

    @Override
    public void loop() {

        // Drive
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        lf.setPower((y + x + rx * 0.8) * powerMult);
        rf.setPower((y - x - rx * 0.8) * powerMult);
        lr.setPower((y - x + rx * 0.8) * powerMult);
        rr.setPower((y + x - rx * 0.8) * powerMult);

        // Intake
        if (gamepad1.left_bumper) {
            intake.setPower(1);
        } else if (gamepad1.left_trigger > 0.1) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        // Shooter velocity selector
        if (gamepad1.triangle) {
            curTargetVelocity = 1040;
        } else if (gamepad1.square) {
            curTargetVelocity = 1250;
        } else if (gamepad1.cross) {
            curTargetVelocity = 1500;
        } else if (gamepad1.right_bumper) {
            curTargetVelocity = 1170;
        } else if (gamepad1.right_trigger > 0.1) {
            curTargetVelocity = -1080;
        } else {
            curTargetVelocity = 0;
        }

        shooter1.setVelocity(curTargetVelocity);
        shooter2.setVelocity(curTargetVelocity);

        double error = shooter1.getVelocity() - curTargetVelocity;

        if (curTargetVelocity > 0 && Math.abs(error) < 50) {
            gamepad1.rumbleBlips(2);
        }

        // Toggle Limelight control
        if (gamepad1.dpadUpWasPressed()) toggle = 1;
        if (gamepad1.dpadDownWasPressed()) toggle = 0;

        // Manual turret mode
        if (toggle == 0) {
            if (gamepad1.dpad_left) {
                turret.setPower(0.3);
            } else if (gamepad1.dpad_right) {
                turret.setPower(-0.3);
            } else {
                turret.setPower(0);
            }
        }

        // Limelight auto-aim mode
        if (toggle == 1) {

            LLStatus status = limelight.getStatus();
            telemetry.addData("LL Name", status.getName());
            telemetry.addData("Temp/CPU/FPS",
                    "%.1fC | %.1f%% | %d",
                    status.getTemp(),
                    status.getCpu(),
                    (int) status.getFps());

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                tx = result.getTx();

                double errorChange = tx - lastError;
                double dt = timer.seconds();
                double derivative = 0;

                if (dt > 0.001) {
                    derivative = errorChange / dt;
                }

                double motorPower = (tx * kP) + (derivative * kD);

                motorPower = Math.max(-0.5, Math.min(0.5, motorPower));

                if (Math.abs(tx) < 3) {
                    turret.setPower(0);
                } else {
                    turret.setPower(motorPower);
                }

                lastError = tx;
                timer.reset();

                telemetry.addData("Target X", tx);
                telemetry.addData("Turret Power", motorPower);

            } else {
                turret.setPower(0);
                telemetry.addLine("No Limelight Target");
            }
        }

        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", shooter1.getVelocity());
        telemetry.update();
    }
}