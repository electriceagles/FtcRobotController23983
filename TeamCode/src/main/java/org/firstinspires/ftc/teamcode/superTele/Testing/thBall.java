package org.firstinspires.ftc.teamcode.superTele.Testing;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "3 ball (blue) no odo", group = "Autonomous")
public class thBall extends LinearOpMode {

    DcMotorEx lf, lr, rf, rr;

    public DcMotorEx intake;

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
    public void runOpMode() {

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

        waitForStart();
        limelight.start();

        if (isStopRequested()) return;

        // ===== DRIVE FORWARD =====
        lf.setPower(-0.4);
        lr.setPower(-0.4);
        rf.setPower(-0.4);
        rr.setPower(-0.4);

        sleep(1100); // drive for 4 seconds

        // ===== STOP =====
        lf.setPower(0);
        lr.setPower(0);
        rf.setPower(0);
        rr.setPower(0);

        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        shooter1.setVelocity(1200);
        shooter2.setVelocity(1200);

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

        sleep(5500);

        intake.setPower(-1);
        sleep(8000);
        intake.setPower(0);
        shooter1.setVelocity(0);
        shooter2.setVelocity(0);
    }
}
