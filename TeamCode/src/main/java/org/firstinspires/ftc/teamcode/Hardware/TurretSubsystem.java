package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Config
@Configurable
public class TurretSubsystem {

    public RobotHardware hardware = new RobotHardware();

    public double targetTicks;
    public long currentTime;
    public double derivative;
    public double lastError;
    public double dt;
    public long lastTime;

    public static double kP = 0.006; // TODO: Tune these
    public static double kD = 0.00006;

    public double TURRET_GEAR_RATIO = 4; // CALCULATE THIS
    public double MOTOR_GEAR_RATIO = 19.2; //TODO: CALCULATE THIS
    public double TICKS_PER_REVOLUTION = 28 * TURRET_GEAR_RATIO * MOTOR_GEAR_RATIO;

    public double getAngleInDegrees() {
        int turretPos = hardware.turret.getCurrentPosition();
        return (turretPos / TICKS_PER_REVOLUTION) * 360;
    }

    public void setRotationalTarget(double thetaDegrees) {
        targetTicks = (thetaDegrees / 360) * TICKS_PER_REVOLUTION;
    }

    public void updatePID(double currentPos) {
        double error = targetTicks - hardware.turret.getCurrentPosition();

        currentTime = System.nanoTime();
        dt = (currentPos - lastError) / 1e9;
        derivative = (error - lastError) / dt;

        if (Math.abs(error) < 2.5) hardware.turret.setPower(0);

        hardware.turret.setPower((kP * error) + (kD * derivative));

        lastError = error;
        lastTime = currentTime;
    }

}
