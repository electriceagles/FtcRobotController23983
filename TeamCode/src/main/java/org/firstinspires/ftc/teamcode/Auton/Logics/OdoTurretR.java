/*package org.firstinspires.ftc.teamcode.Auton.Logics;


import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;


import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Hardware.TurretSubsystem;

public class OdoTurretR {
    final double xTarget = 133;
    final double yTarget = 138;
    public DcMotorEx turret;
    GoBildaPinpointDriver pinpoint;
    TurretSubsystem turretSubsystem;


    public void init(HardwareMap hw) {
    turret = hw.get(DcMotorEx.class, "turret");
    turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    turret.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    turretSubsystem = new TurretSubsystem();
    turretSubsystem.hardware.turret = turret;

    pinpoint = hw.get(GoBildaPinpointDriver.class, "odo");
    pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
    pinpoint.setOffsets(-8, 0.5, DistanceUnit.INCH); // offset for pods (might have to change)

    //field start pose (inches)
    pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 56, 8, AngleUnit.DEGREES, 90));
    }

    public void loop(){
        pinpoint.update();

        Pose2D pose = pinpoint.getPosition();

        double xRobot = pose.getX(DistanceUnit.INCH);
        double yRobot = pose.getY(DistanceUnit.INCH);
        double heading = pose.getHeading(AngleUnit.RADIANS);

        //calculation to align turret with target
        double targetDirection = Math.atan2(xTarget - xRobot, yTarget - yRobot);
        double turretTargetAngle = AngleUnit.normalizeRadians(targetDirection - heading);

        turretSubsystem.setRotationalTarget(turretTargetAngle);
        turretSubsystem.updatePID(turret.getCurrentPosition());
    }
}
*/