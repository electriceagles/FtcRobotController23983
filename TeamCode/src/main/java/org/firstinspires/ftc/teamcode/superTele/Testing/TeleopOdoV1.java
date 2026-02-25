package org.firstinspires.ftc.teamcode.superTele.Testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;


@TeleOp(name = "TeleOP Odo Turret BLUE v1", group = "TeleOp")
public class TeleopOdoV1 extends OpMode {

    // Drive motors
    DcMotorEx lf, lr, rf, rr;

    // Mechanisms
    DcMotorEx intake;
    DcMotorEx turret;
    DcMotorEx shooter1, shooter2;

    Servo gate;

    CRServo gekko;

    private Limelight3A limelight;
    public double tx = 0;
    public double ty = 0;

    public double lastError = 0;
    public double kP = 0.1567; //change based on onbot java values
    public double kD = 0.08;

    ElapsedTime timer = new ElapsedTime();

    // Odometry
    GoBildaPinpointDriver pinpoint;

    // Shooter tuning
    double P = 111.40100;
    double F = 14.05;
    double curTargetVelocity = 0;

    // Turret constants
    final double TICKS_PER_REV = 2150.8;
    final double TICKS_PER_RAD = TICKS_PER_REV / (2 * Math.PI);

    // field target (in inches for red)
    final double xTarget = 133;
    final double yTarget = 138;

    double powerMult = 0.9;

    boolean visionTurret = false;

    @Override
    public void init() {
        //limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);

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
        gate = hardwareMap.get(Servo.class, "gate");
        gekko = hardwareMap.get(CRServo.class, "gekko");

        // turret
        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.65);

        // shooter
        shooter1 = hardwareMap.get(DcMotorEx.class, "sf1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "sf2");

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(P, 0, 0, F);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        // pinpoint and odometry
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(-84, -168, DistanceUnit.MM); // offset for pods (might have to change)

        //field start pose (inches)
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 56, 8, AngleUnit.DEGREES, 90));

        telemetry.addLine("Starting Pose: 56, 8, 90º");
    }

    @Override
    public void start() {
        limelight.start();
    }

    @Override
    public void loop() {

       //drive
        double y = gamepad1.left_stick_y;
        double x = -gamepad1.left_stick_x;
        double rx = -gamepad1.right_stick_x;

        lf.setPower((y + x + rx * 0.8) * powerMult);
        rf.setPower((y - x - rx * 0.8) * powerMult);
        lr.setPower((y - x + rx * 0.8) * powerMult);
        rr.setPower((y + x - rx * 0.8) * powerMult);

        // intake or transfer
        if (gamepad1.dpad_up) {
            intake.setPower(1);
            gekko.setPower(-1);
        } else if (gamepad1.left_bumper) {
            gate.setPosition(23); // tune
            intake.setPower(-1);
        } else if (gamepad1.left_trigger > 0.1) {
            intake.setPower(-1);
            gekko.setPower(-1);
        } else {
            intake.setPower(0);
            gate.setPosition(0); //tune
            gekko.setPower(0);
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

        if (curTargetVelocity > 0 && Math.abs(error1) == 50){
            gamepad1.rumbleBlips(2);
        }

        //odometry update and data
        pinpoint.update();

        Pose2D pose = pinpoint.getPosition();

        double xRobot = pose.getX(DistanceUnit.INCH);
        double yRobot = pose.getY(DistanceUnit.INCH);
        double heading = pose.getHeading(AngleUnit.RADIANS);

        //calculation to align turret with target
        double targetDirection = Math.atan2(xTarget - xRobot, yTarget - yRobot);
        double turretTargetAngle = AngleUnit.normalizeRadians(targetDirection - heading);
        int targetTicks = (int)(turretTargetAngle * TICKS_PER_RAD);

        if (gamepad1.right_bumper || gamepad1.triangle || gamepad1.square || gamepad1.cross) {
            turret.setTargetPosition(targetTicks);
        }


        //switching to vision turret aim (failsafe)
        if (gamepad1.dpad_left) {
            visionTurret = true;
        } else {
            visionTurret = false;
        }

        //limelight visionTurret function
        if (visionTurret) {
            LLStatus status = limelight.getStatus();
            telemetry.addData("Name", "%s",
                    status.getName());
            telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                    status.getTemp(), status.getCpu(), (int) status.getFps());
            telemetry.addData("Pipeline", "Index: %d, Type: %s",
                    status.getPipelineIndex(), status.getPipelineType());

            //limelight tag aim
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                tx = result.getTx();
                ty = result.getTy();

                double errorChange = tx - lastError;

                double derivative = errorChange / timer.seconds();

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
                telemetry.addData("power", motorPower);
            } else {
                turret.setPower(0);
                telemetry.addData("Limelight", "No Targets");
            }
        }

        //telemetry
        telemetry.addData("Robot X", xRobot);
        telemetry.addData("Robot Y", yRobot);
        telemetry.addData("Heading", heading);
        telemetry.addData("Turret Target Ticks", targetTicks);
        telemetry.addData("Target Velocity", curTargetVelocity);
        telemetry.addData("Current Velocity", shooter1.getVelocity());
        telemetry.update();

    }
}