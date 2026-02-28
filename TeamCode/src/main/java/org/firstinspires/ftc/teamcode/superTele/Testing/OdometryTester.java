package org.firstinspires.ftc.teamcode.superTele.Testing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.TurretSubsystem;

@Configurable
@TeleOp
public class OdometryTester extends OpMode {

    GoBildaPinpointDriver pinpoint;

    DcMotorEx turret;


    final double xTarget = 133;
    final double yTarget = 138;

    private static double turretToggle = 0;

    private TurretSubsystem turretSubsystem;


    @Override
    public void init() {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odo");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(-84, -168, DistanceUnit.MM);

        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 56, 8, AngleUnit.DEGREES, 90));

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turretSubsystem = new TurretSubsystem();
        turretSubsystem.hardware.turret = turret;
    }

    @Override
    public void loop() {

        pinpoint.update();

        Pose2D pose = pinpoint.getPosition();

        double xRobot = pose.getX(DistanceUnit.INCH);
        double yRobot = pose.getY(DistanceUnit.INCH);
        double heading = pose.getHeading(AngleUnit.RADIANS);

        //calculation to align turret with target
        double targetDirection = Math.atan2(yTarget - yRobot, xTarget - xRobot);
        double turretTargetAngle = AngleUnit.normalizeRadians(targetDirection - heading);

        turretSubsystem.setRotationalTarget(turretTargetAngle);


        if (turretToggle == 1) {
            turretSubsystem.updatePID(turret.getCurrentPosition());
        }



        telemetry.addData("x", xRobot);
        telemetry.addData("y", yRobot);
        telemetry.addData("heading", heading);
        telemetry.addLine("----------------------------------------------");
        telemetry.addData("angle diff to target", turretTargetAngle);
        telemetry.update();

    }
}
