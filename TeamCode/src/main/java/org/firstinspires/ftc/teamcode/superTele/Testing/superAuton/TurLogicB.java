package org.firstinspires.ftc.teamcode.superTele.Testing.superAuton;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.*;
import org.firstinspires.ftc.robotcore.external.navigation.*;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.util.ElapsedTime;
public class TurLogicB {
    DcMotorEx turret;
    public double kP = 0.1567; //change based on onbot java values
    public double kD = 0.08;

    ElapsedTime timer = new ElapsedTime();

    // Odometry
    GoBildaPinpointDriver pinpoint;

    // Turret constants
    final double TICKS_PER_REV = 2150.8;
    final double TICKS_PER_RAD = TICKS_PER_REV / (2 * Math.PI);

    // field target (in inches for blue)
    final double xTarget = 0;
    final double yTarget = 144;

    public void init (HardwareMap hwMap) {
        turret = hwMap.get(DcMotorEx.class, "turret");
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setTargetPosition(0);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(0.65);
    }

}
