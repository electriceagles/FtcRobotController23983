package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "basicDrive", group = "TeleOp")
public class basicDrive extends LinearOpMode {
    private DcMotor rightBack, rightFront, leftFront, leftBack;

    @Override
    public void runOpMode() {
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");

        leftBack.setDirection(DcMotor.Direction.REVERSE);
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_stick_y < -0.1) {
                rightBack.setPower(-0.6);
                rightFront.setPower(-0.6);
                leftBack.setPower(-0.6);
                leftFront.setPower(-0.6);
            } else if (gamepad1.left_stick_y > 0.1) {
                rightBack.setPower(0.6);
                rightFront.setPower(0.6);
                leftBack.setPower(0.6);
                leftFront.setPower(0.6);
            } else if (gamepad1.left_stick_x < -0.1) {
                rightBack.setPower(-0.6);
                rightFront.setPower(0.6);
                leftBack.setPower(0.6);
                leftFront.setPower(-0.6);
            } else if (gamepad1.left_stick_x > 0.1) {
                rightBack.setPower(0.6);
                rightFront.setPower(-0.6);
                leftBack.setPower(-0.6);
                leftFront.setPower(0.6);
            } else {
                rightBack.setPower(0);
                rightFront.setPower(0);
                leftBack.setPower(0);
                leftFront.setPower(0.0);
            }
        }

    }
}