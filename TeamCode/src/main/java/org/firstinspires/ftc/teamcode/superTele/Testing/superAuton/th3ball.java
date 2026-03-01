package org.firstinspires.ftc.teamcode.superTele.Testing.superAuton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

@Autonomous(name = "6 ball")
public class th3ball extends LinearOpMode {

    DcMotorEx lf, lr, rf, rr;
    DcMotorEx turret;

    // Mechanisms
    DcMotorEx intake;
    DcMotorEx shooter1, shooter2;

    @Override
    public void runOpMode() throws InterruptedException {

        // Drive motors
        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lr = hardwareMap.get(DcMotorEx.class, "lr");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rr = hardwareMap.get(DcMotorEx.class, "rr");

        turret = hardwareMap.get(DcMotorEx.class, "turret");

        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lr.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        // Intake
        intake = hardwareMap.get(DcMotorEx.class, "i");

        // Shooters
        shooter1 = hardwareMap.get(DcMotorEx.class, "sf1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "sf2");

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDFCoefficients pidf = new PIDFCoefficients(111.401, 0, 0, 14.05);
        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidf);

        waitForStart();

        if (opModeIsActive()) {

            // Drive forward
            lf.setPower(0.7);
            lr.setPower(0.7);
            rf.setPower(0.7);
            rr.setPower(0.7);
            sleep(500);

            stopDrive();

            // Spin up shooters
            shooter1.setVelocity(1160);
            shooter2.setVelocity(1160);
            sleep(3000);

            // Run intake
            intake.setPower(-1);
            sleep(8000);

            shooter1.setVelocity(0);
            shooter2.setVelocity(0);
            intake.setPower(0);

            // Turn ~45 degrees
            lf.setPower(-0.5);
            lr.setPower(-0.5);
            rf.setPower(0.5);
            rr.setPower(0.5);
            sleep(140);

            stopDrive();

            //intake


            lf.setPower(-0.7);
            lr.setPower(-0.7);
            rf.setPower(-0.7);
            rr.setPower(-0.7);
            sleep(4500);

            stopDrive();

            intake.setPower(-1);
            sleep(2500);

            // Drive backward with intake
            lf.setPower(0.7);
            lr.setPower(0.7);
            rf.setPower(0.7);
            rr.setPower(0.7);
            sleep(4500);

            stopDrive();

            lf.setPower(0.5);
            lr.setPower(0.5);
            rf.setPower(-0.5);
            rr.setPower(-0.5);
            sleep(140);

            stopDrive();



            // Final shooter spin
            shooter1.setVelocity(1160);
            shooter2.setVelocity(1160);
            sleep(3000);

            // Run intake
            intake.setPower(-1);
            sleep(8000);

            shooter1.setVelocity(0);
            shooter2.setVelocity(0);
            intake.setPower(0);

            stopDrive();

            sleep(10000);
        }
    }

    private void stopDrive() {
        lf.setPower(0);
        lr.setPower(0);
        rf.setPower(0);
        rr.setPower(0);
    }
}