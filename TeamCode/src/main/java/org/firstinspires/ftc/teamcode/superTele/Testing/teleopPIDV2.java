package org.firstinspires.ftc.teamcode.superTele.Testing;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp (name = "TeleOP + PID v2 (Blue aim)", group = "TeleOp")
public class teleopPIDV2 extends OpMode {

    public DcMotorEx lf, lr, rf, rr;

    public DcMotorEx intake;
    public double powerMult = 0.9;

    double curTargetVelocity= 0;

    private Limelight3A limelight;

    private DcMotorEx turret;

    public double tx = 0;
    public double ty = 0;

    public double lastError = 0;
    public double kP = 0.1567;
    public double kD = 0.08;

    ElapsedTime timer = new ElapsedTime();

    double P = 111.40100;
    double F = 14.05;


    public DcMotorEx shooter1;
    public DcMotorEx shooter2;


    @Override
    public void init(){

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorEx.Direction.REVERSE);


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
    public void start() {
        limelight.start();

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
        if (gamepad1.left_bumper) {
            intake.setPower(1);
        } else if (gamepad1.left_trigger > 0.1) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }


        //set and or change shooter RPM
        if (gamepad1.triangle) {
            curTargetVelocity = 1040;
        } else if (gamepad1.square) {
            curTargetVelocity = 1120;
        } else if (gamepad1.cross) {
            curTargetVelocity = 1580;
        } else if (gamepad1.right_bumper) {
            curTargetVelocity = 1200;//myles changed this value from 1360 to 1200 temporarily
        } else {
            curTargetVelocity = 0;
        }

        //set PIDF coef

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(P, 0, 0, F);

        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        //set chosen velocity
        shooter1.setVelocity(curTargetVelocity);
        shooter2.setVelocity(curTargetVelocity);

        //limelight data
        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(), (int) status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());

        //tag aim
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

        double curVelocity1 = shooter1.getVelocity();

        double error1 = curVelocity1 - curTargetVelocity;

        if (gamepad1.right_bumper && Math.abs(error1) == 50){
            gamepad1.rumbleBlips(2);
        }

        //telemetry stuff


        telemetry.addData("Selected Velocity", curTargetVelocity);
        telemetry.addData("P", "%.4f", P);
        telemetry.addData("F", "%.4f", F);
        telemetry.addData("turret P", "%.4f", kP);
        telemetry.addData("turret D", "%.4f", kD);
    }
}