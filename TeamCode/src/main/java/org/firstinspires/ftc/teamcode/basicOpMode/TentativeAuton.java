package org.firstinspires.ftc.teamcode.basicOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Simple Leave and Return", group = "Autonomous")
public class TentativeAuton extends LinearOpMode {

    DcMotorEx lf, lr, rf, rr;
    GoBildaPinpointDriver odo;

    @Override
    public void runOpMode() {

        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lr = hardwareMap.get(DcMotorEx.class, "lr");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rr = hardwareMap.get(DcMotorEx.class, "rr");

        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lr.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "imu");
        odo.resetPosAndIMU();

        waitForStart();
        if (isStopRequested()) return;

        // drive ahead 15 inch
        while (opModeIsActive() && odo.getPosY(DistanceUnit.INCH) < 15) {
            odo.update();

            lf.setPower(0.4);
            lr.setPower(0.4);
            rf.setPower(0.4);
            rr.setPower(0.4);
        }

        stopDrive();
        sleep(300);

        // back to base
        while (opModeIsActive() && odo.getPosY(DistanceUnit.INCH) > 0) {

            odo.update();

            lf.setPower(-0.4);
            lr.setPower(-0.4);
            rf.setPower(-0.4);
            rr.setPower(-0.4);
        }

        stopDrive();
    }

    void stopDrive() {
        lf.setPower(0);
        lr.setPower(0);
        rf.setPower(0);
        rr.setPower(0);
    }
}