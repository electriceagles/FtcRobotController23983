package org.firstinspires.ftc.teamcode.superTele.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.Hardware.TurretSubsystem;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
@TeleOp(name = "TeleOP Odo Turret PID", group = "TeleOp")
public class TeleOpOdov2 extends LinearOpMode {

    private DcMotorEx turret;
    private TurretSubsystem turretSubsystem;

    @Override
    public void runOpMode() throws InterruptedException {

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turretSubsystem = new TurretSubsystem();
        turretSubsystem.hardware.turret = turret;

        waitForStart();

        while (opModeIsActive()) {

            turretSubsystem.setRotationalTarget(-45);

            turretSubsystem.updatePID(turret.getCurrentPosition());

            telemetry.addData("Angle (deg)", turretSubsystem.getAngleInDegrees());
            telemetry.addData("Target Ticks", turretSubsystem.targetTicks);
            telemetry.update();
        }
    }
}