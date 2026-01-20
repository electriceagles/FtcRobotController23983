package org.firstinspires.ftc.teamcode.basicOpMode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

public class FlywheelLogic {

    private DcMotorEx shooterFlyWheel1, shooterFlyWheel2;

    private static final double TICKS_PER_REV = 28.0;
    private static final double LOOP_DT = 0.02; // 20ms

    private double targetRPM = 0;

    // PID
    private double kP = 0.00035;
    private double kI = 0.0000008;
    private double kD = 0.00012;

    private double integral = 0;
    private double lastError = 0;

    private int lastTicks = 0;
    private double lastTime = 0;

    public void init(HardwareMap hw) {
        shooterFlyWheel1 = hw.get(DcMotorEx.class, "sf1");
        shooterFlyWheel2 = hw.get(DcMotorEx.class, "sf2");

        shooterFlyWheel1.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooterFlyWheel2.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        shooterFlyWheel1.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooterFlyWheel2.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        shooterFlyWheel1.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        shooterFlyWheel2.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        shooterFlyWheel1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterFlyWheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        lastTicks = shooterFlyWheel1.getCurrentPosition();
        lastTime = System.nanoTime() / 1e9;
    }

    public void setTargetRPM(double rpm) {
        targetRPM = rpm;
    }

    public void stop() {
        targetRPM = 0;
        shooterFlyWheel1.setPower(0);
        shooterFlyWheel2.setPower(0);
        integral = 0;
        lastError = 0;
    }

    public boolean atSpeed() {
        return Math.abs(getRPM() - targetRPM) < 75;
    }

    public void update() {
        if (targetRPM <= 0) return;

        double rpm = getRPM();
        double error = targetRPM - rpm;

        integral += error * LOOP_DT;
        integral = Range.clip(integral, -5000, 5000);

        double derivative = (error - lastError) / LOOP_DT;
        lastError = error;

        double power =
                kP * error +
                        kI * integral +
                        kD * derivative;

        power = Range.clip(power, 0, 1);

        shooterFlyWheel1.setPower(power);
        shooterFlyWheel2.setPower(power);
    }

    private double getRPM() {
        int ticks = shooterFlyWheel1.getCurrentPosition();
        double time = System.nanoTime() / 1e9;

        double dt = time - lastTime;
        if (dt <= 0) return 0;

        double rpm = ((ticks - lastTicks) / TICKS_PER_REV) / dt * 60.0;

        lastTicks = ticks;
        lastTime = time;

        return Math.abs(rpm);
    }
}
