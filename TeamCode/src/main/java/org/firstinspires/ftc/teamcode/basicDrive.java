package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp(name = "basicDrive", group = "TeleOp")
public class basicDrive extends LinearOpMode {
    private DcMotor leftDrive;
    private DcMotor rightDrive;

    @Override
    public void runOpMode() {
        leftDrive = hardwareMap.get(DcMotor.class, "leftDrive");
        rightDrive = hardwareMap.get(DcMotor.class, "rightDrive");

        leftDrive.setDirection(DcMotor.Direction.REVERSE);
        rightDrive.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.left_stick_y < -0.1) {
                leftDrive.setPower(0.6);
                rightDrive.setPower(0.6);
            } else {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            }

            if (gamepad1.left_stick_y > 0.1) {
                leftDrive.setPower(-0.6);
                rightDrive.setPower(-0.6);
            } else {
                leftDrive.setPower(0);
                rightDrive.setPower(0);
            }
        }

    }
}


