package org.firstinspires.ftc.teamcode.basicOpMode.TeaAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

@Disabled // Disabled it for now since I cannot test it, If someone does plan to test this
// Record a video of the test and remove the @Disabled and remember to push code back to the control hub
@Autonomous
public class EncoderAutoBlueObelisk extends LinearOpMode {

    public RobotHardware hardware = new RobotHardware();

    public static final double TICKS_PER_REV = 383.6;
    public static final double WHEEL_DIAMETER = 4.09;  // Inches
    public static final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER);

    public static final double TRACK_WIDTH = 12; // Inches

    @Override
    public void runOpMode() throws InterruptedException {

        /** even tho this is auto, I don't want to use roadrunner for this auto **/
        hardware.init(hardwareMap, false);
        hardware.resetEnc();

        waitForStart();

        while (!isStarted()) {
            // TODO: Display telemetry here.
        }

        if (opModeIsActive()) {

            // This is a test if this works we can have this as a backup auto in case you don't have enough time to tune.
            drive(2000, 0.5);
            sleep(2000);
            turn(-90, 0.5);
            sleep(1000);
            strafe("left", 500, 0.5);
        }

    }


    public void drive(int encPos, double power) {
        hardware.resetEnc();

        hardware.leftFront.setTargetPosition(encPos);
        hardware.rightFront.setTargetPosition(encPos);
        hardware.rightRear.setTargetPosition(encPos);
        hardware.leftRear.setTargetPosition(encPos);

        hardware.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hardware.leftFront.setPower(power);
        hardware.rightFront.setPower(power);
        hardware.rightRear.setPower(power);
        hardware.leftRear.setPower(power);
    }

    public void turn(int degrees, double power) {
        hardware.resetEnc();
        double turnInches = Math.PI * TRACK_WIDTH * (degrees / 360.0);

        int turnTicks = (int) (turnInches * TICKS_PER_INCH);

        hardware.leftRear.setTargetPosition(hardware.leftRear.getCurrentPosition() + turnTicks);
        hardware.rightRear.setTargetPosition(hardware.rightRear.getCurrentPosition() - turnTicks);
        hardware.leftFront.setTargetPosition(hardware.leftFront.getCurrentPosition() + turnTicks);
        hardware.rightFront.setTargetPosition(hardware.rightFront.getCurrentPosition() - turnTicks);

        hardware.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        hardware.leftFront.setPower(power);
        hardware.rightFront.setPower(power);
        hardware.rightRear.setPower(power);
        hardware.leftRear.setPower(power);
    }

    public void strafe(String direction, int encPos, double power) {
        hardware.resetEnc();

        hardware.leftFront.setTargetPosition(encPos);
        hardware.rightFront.setTargetPosition(encPos);
        hardware.rightRear.setTargetPosition(encPos);
        hardware.leftRear.setTargetPosition(encPos);

        hardware.leftRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.leftFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.rightFront.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        hardware.rightRear.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (direction.equals("left")) {
            hardware.leftFront.setPower(-power);
            hardware.rightFront.setPower(power);
            hardware.rightRear.setPower(-power);
            hardware.leftRear.setPower(power);
        } else if (direction.equals("right")) {
            hardware.leftFront.setPower(power);
            hardware.rightFront.setPower(-power);
            hardware.rightRear.setPower(power);
            hardware.leftRear.setPower(-power);
        }
    }
}
