package org.firstinspires.ftc.teamcode.Q2BasicAutons;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Leave and Return (v2)", group = "Autonomous")
public class TimedAuton extends LinearOpMode {

    DcMotorEx lf, lr, rf, rr;

    @Override
    public void runOpMode() {

        // ---- Drive motors ----
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lr = hardwareMap.get(DcMotorEx.class, "lr");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rr = hardwareMap.get(DcMotorEx.class, "rr");

        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lr.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        if (isStopRequested()) return;

        // ================= DRIVE FORWARD =================
        lf.setPower(0.4);
        lr.setPower(0.4);
        rf.setPower(0.4);
        rr.setPower(0.4);

        sleep(1100); // drive forward for 3 seconds

        stopDrive();
        sleep(500);

        // ================= DRIVE BACK =================
        lf.setPower(-0.4);
        lr.setPower(-0.4);
        rf.setPower(-0.4);
        rr.setPower(-0.4);

        sleep(670); // drive backward for 3 seconds

        stopDrive();
    }

    void stopDrive() {
        lf.setPower(0);
        lr.setPower(0);
        rf.setPower(0);
        rr.setPower(0);
    }
}
