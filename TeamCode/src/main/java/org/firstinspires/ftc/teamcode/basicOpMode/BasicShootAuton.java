//**note for tamzid: tune the sleep values for shooting part**

package org.firstinspires.ftc.teamcode.basicOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Simple Leave, shoot and Return", group = "Autonomous")
public class BasicShootAuton extends LinearOpMode {

    DcMotorEx lf, lr, rf, rr;

    public DcMotorEx intake;

    public DcMotorEx turret;
    public DcMotorEx shooter1, shooter2;
    GoBildaPinpointDriver odo;


    @Override
    public void runOpMode() {

        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lr = hardwareMap.get(DcMotorEx.class, "lr");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rr = hardwareMap.get(DcMotorEx.class, "rr");

        intake = hardwareMap.get(DcMotorEx.class, "i");
        shooter1 = hardwareMap.get(DcMotorEx.class, "sf1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "sf2");

        turret = hardwareMap.get(DcMotorEx.class, "turret");

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lr.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        odo = hardwareMap.get(GoBildaPinpointDriver.class, "imu");
        odo.resetPosAndIMU();

        waitForStart();
        if (isStopRequested()) return;

        // drive ahead 15 inch
        while (opModeIsActive()) {
            odo.update();

            if (odo.getPosX(DistanceUnit.INCH) >= 30) break;

            lf.setPower(0.4);
            lr.setPower(0.4);
            rf.setPower(0.4);
            rr.setPower(0.4);
        }

        stopDrive();
        sleep(1700);

        turret.setPower(-0.8); //turret moves right
        sleep(1000); // for 1 sec (by now it hopefully aligns)
        turret.setPower(0); //turret pause and stays still

        shooter1.setPower(0.7); //shooter revs up
        shooter2.setPower(0.7);

        intake.setPower(-1);
        sleep(2000);
        intake.setPower(0);
        sleep(2500);
        intake.setPower(-1);
        sleep(2000);
        intake.setPower(0);
        sleep(2500);
        intake.setPower(-1);
        sleep(2000);
        intake.setPower(0);
        sleep(500);

        shooter1.setPower(0);
        shooter2.setPower(0);

        // back to base
        while (opModeIsActive()) {
            odo.update();

            if (odo.getPosX(DistanceUnit.INCH) <= 0) break;

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