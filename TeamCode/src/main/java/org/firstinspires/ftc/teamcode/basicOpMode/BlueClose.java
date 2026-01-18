package org.firstinspires.ftc.teamcode.basicOpMode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "Leave + Strafe Left (Blue)", group = "Autonomous")
public class BlueClose extends LinearOpMode {

    DcMotorEx lf, lr, rf, rr;

    @Override
    public void runOpMode() {

        // Drive motors
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

        // ===== DRIVE BACKWARD =====
        lf.setPower(-0.4);
        lr.setPower(-0.4);
        rf.setPower(-0.4);
        rr.setPower(-0.4);

        sleep(3000); // adjust if needed

        // ===== STRAFE LEFT =====
        // Mecanum strafe left:
        lf.setPower(-0.4);
        lr.setPower(0.4);
        rf.setPower(0.4);
        rr.setPower(-0.4);

        sleep(200); // 0.2 seconds

        // ===== STOP =====
        lf.setPower(0);
        lr.setPower(0);
        rf.setPower(0);
        rr.setPower(0);
    }
}