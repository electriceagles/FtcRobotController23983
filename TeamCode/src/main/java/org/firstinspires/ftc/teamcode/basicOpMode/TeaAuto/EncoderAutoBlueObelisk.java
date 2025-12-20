package org.firstinspires.ftc.teamcode.basicOpMode.TeaAuto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

/*@Disabled Disabled it for now since I cannot test it, If someone does plan to test this
Record a video of the test and remove the @Disabled and remember to push code back to the control hub
 */
@Autonomous
public class EncoderAutoBlueObelisk extends LinearOpMode {

    public RobotHardware hardware = new RobotHardware();

    public static final double TICKS_PER_REV = 383.6;
    public static final double WHEEL_DIAMETER = 4.09;  // Inches
    public static final double TICKS_PER_INCH = TICKS_PER_REV / (Math.PI * WHEEL_DIAMETER);

    public static final double TRACK_WIDTH = 12; // Inches

    @Override
    public void runOpMode() throws InterruptedException {

//        /** even tho this is auto, I don't want to use roadrunner for this auto **/
        hardware.init(hardwareMap, false);
        hardware.resetEnc();

        waitForStart();

        while (!isStarted()) {
            // TODO: Display telemetry here.
        }

        if (opModeIsActive()) {

            // This is a test if this works we can have this as a backup auto in case you don't have enough time to tune.
            driveInches(72, 0.5);
            sleep(2000);
            turn(-90, 0.5);
            sleep(1000);
            servo(0);
            sleep(1000);
            rev(0.67, 6700);
            sleep(1000);
            servo(0.5);
            sleep(1000);

        }

    }

    public void driveInches(double inches, double power) {
        int ticks = (int) Math.round(inches * TICKS_PER_INCH);
        drive(ticks, power);
    }
    private void waitForDriveMotors() {
        while (opModeIsActive()
                && (hardware.leftFront.isBusy()
                || hardware.rightFront.isBusy()
                || hardware.leftRear.isBusy()
                || hardware.rightRear.isBusy())) {
            idle();
        }

        // stop motors after reaching target
        hardware.leftFront.setPower(0);
        hardware.rightFront.setPower(0);
        hardware.leftRear.setPower(0);
        hardware.rightRear.setPower(0);
    }

    public void rev(double power, double time){
        hardware.resetEnc();

        hardware.shooterFlyWheel1.setPower(power);
        hardware.shooterFlyWheel2.setPower(power);

       sleep((long)(time));

       hardware.shooterFlyWheel1.setPower(0);
       hardware.shooterFlyWheel2.setPower(0);
    }

    public void servo(double position) {
        hardware.resetEnc();

        hardware.servo.setPosition(position);
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

        waitForDriveMotors();
    }

    public void turret(){
        hardware.resetEnc();




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

        waitForDriveMotors();
    }
    public void strafeInches(String direction, double inches, double power) {
        int ticks = (int) Math.round(inches * TICKS_PER_INCH);
        strafe(direction, ticks, power);
        waitForDriveMotors();
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
