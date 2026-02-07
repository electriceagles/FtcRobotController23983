package org.firstinspires.ftc.teamcode.LimelightTest;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Config
@TeleOp(name="Test Limelight PID")
public class PIDturret extends OpMode {

    private Limelight3A limelight;

    private DcMotorEx turret;

    double tx = 0;
    double ty = 0;

    double lastError = 0;
    public static double kP = 0.04;
    public static double kD = 0.008;



    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(2);

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setDirection(DcMotorEx.Direction.REVERSE);
    }

    @Override
    public void start() {
        limelight.start();

    }

    @Override
    public void loop() {
        LLStatus status = limelight.getStatus();
        telemetry.addData("Name", "%s",
                status.getName());
        telemetry.addData("LL", "Temp: %.1fC, CPU: %.1f%%, FPS: %d",
                status.getTemp(), status.getCpu(), (int) status.getFps());
        telemetry.addData("Pipeline", "Index: %d, Type: %s",
                status.getPipelineIndex(), status.getPipelineType());


        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            tx = result.getTx();
            ty = result.getTy();

            double errorChange = tx - lastError;

            double motorPower = (tx * kP) + (errorChange * kD);

            if (motorPower > 1.0) {
                motorPower = 1.0;
            }
            if (motorPower < -1.0) {
                motorPower = -1.0;
            }

            if (Math.abs(tx) < 3) {
                turret.setPower(0);
            } else {
                turret.setPower(motorPower);
            }

            lastError = tx;

            telemetry.addData("Target X", tx);
            telemetry.addData("power", motorPower);
        } else {
            turret.setPower(0);
            telemetry.addData("Limelight", "No Targets");
        }

    }
}
