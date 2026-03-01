package org.firstinspires.ftc.teamcode.superTele.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.*;

@Configurable
@TeleOp(name = "Limelight Turret TeleOp", group = "TeleOp")
public class TeleopOdoV1 extends OpMode {

    // Drive motors
    DcMotorEx lf, lr, rf, rr;

    // Mechanisms
    DcMotorEx intake;
    DcMotorEx turret;
    DcMotorEx shooter1, shooter2;

    // Limelight
    private Limelight3A limelight;
    public double tx = 0;
    public double ty = 0;
    public double lastError = 0;

    // PD control constants
    private static double kP = 0.002;
    private static double kD = 0.02;

    private boolean limelightTurret = false;
    private int toggle = 0;
    private ElapsedTime timer = new ElapsedTime();

    // Shooter
    double curTargetVelocity = 0;

    // Odometry
    GoBildaPinpointDriver pinpoint;

    // Power multiplier for drive
    double powerMult = 0.9;

    @Override
    public void init() {
        // Limelight setup
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);

        // Drive motors
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
        turret.setDirection(DcMotorEx.Direction.REVERSE);

        // Shooter
        shooter1 = hardwareMap.get(DcMotorEx.class, "sf1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "sf2");
        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);
        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        PIDFCoefficients pidf = new PIDFCoefficients(111.401, 0, 0, 14.05);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        //
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(-84, -168, DistanceUnit.MM);
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 56, 8, AngleUnit.DEGREES, 90));

        telemetry.addLine("Starting Pose: 56, 8, 90º");
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;
        lf.setPower((y + x + rx * 0.8) * powerMult);
        rf.setPower((y - x - rx * 0.8) * powerMult);
        lr.setPower((y - x + rx * 0.8) * powerMult);
        rr.setPower((y + x - rx * 0.8) * powerMult);

        if (gamepad1.left_bumper) intake.setPower(1);
        else if (gamepad1.left_trigger > 0.1) intake.setPower(-1);
        else intake.setPower(0);

        if (gamepad1.triangle) {
            curTargetVelocity = 1030;
        } else if (gamepad1.square) {
            curTargetVelocity = 1240;
        } else if (gamepad1.cross) {
            curTargetVelocity = 1540;
        } else if (gamepad1.right_bumper) {
            curTargetVelocity = 1170;
        } else if (gamepad1.right_trigger > 0) {
            curTargetVelocity = -1080;
        } else {
            curTargetVelocity = 0;
        }

        shooter1.setVelocity(curTargetVelocity);
        shooter2.setVelocity(curTargetVelocity);

        double curVelocity1 = shooter1.getVelocity();
        double error1 = curVelocity1 - curTargetVelocity;

        if (curTargetVelocity > 0 && Math.abs(error1) < 50) {
            gamepad1.rumbleBlips(2);
        }

        if (gamepad1.dpad_left) {
            turret.setPower(0.3);
        } else if (gamepad1.dpad_right) {
            turret.setPower(-0.3);
        } else {
            turret.setPower(0);
        }

//        if (toggle == 1) {
//                LLResult result = limelight.getLatestResult();
//                if (result != null && result.isValid()) {
//                    tx = result.getTx();
//                    ty = result.getTy();
//
//                    double errorChange = tx - lastError;
//                    double derivative = errorChange / Math.max(timer.seconds(), 0.01); // prevent divide by zero
//                    double motorPower = -((tx * kP) + (derivative * kD));
//
//                    // clamp motor power
//                    motorPower = Math.max(-0.7, Math.min(0.7, motorPower));
//                    turret.setPower(motorPower);
//
//                    lastError = tx;
//                    timer.reset();
//                } else {
//                    turret.setPower(0);
//                }
//            }

            telemetry.addData("Turret Encoder", turret.getCurrentPosition());
            telemetry.addData("Limelight tx", tx);
            telemetry.addData("Shooter Target", curTargetVelocity);
            telemetry.addData("Shooter Velocity", shooter1.getVelocity());
            telemetry.update();
        }
}
