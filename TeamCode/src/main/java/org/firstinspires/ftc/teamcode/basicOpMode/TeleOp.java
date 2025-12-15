package org.firstinspires.ftc.teamcode.basicOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp (name = "Duo TeleOp", group = "TeleOp")
public class TeleOp extends LinearOpMode {

    public DcMotorEx leftFront;
    public DcMotorEx rightRear;
    public DcMotorEx leftRear;
    public DcMotorEx rightFront;

    public DcMotorEx intake;

    public DcMotorEx shooterFlyWheel1;
    public DcMotorEx shooterFlyWheel2;


    public double powerMult = 1;
    public double powerLim = 1;


    @Override
    public void runOpMode() throws InterruptedException {


        // Put your configured names in the blank spaces of ""
        // Make sure they accurately match your configuration
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        rightRear = hardwareMap.get(DcMotorEx.class, "rr");
        leftRear = hardwareMap.get(DcMotorEx.class, "lr");
        rightFront = hardwareMap.get(DcMotorEx.class, "rf");

        intake = hardwareMap.get(DcMotorEx.class, "i");

        shooterFlyWheel1 = hardwareMap.get(DcMotorEx.class, "sf1");
        shooterFlyWheel2 = hardwareMap.get(DcMotorEx.class, "sf2");

        /* First test and see how the motor spins,
        if you want the motor to spin forward then reverse the direction */
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRear.setDirection(DcMotorSimple.Direction.FORWARD);
        rightFront.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRear.setDirection(DcMotorSimple.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);

        shooterFlyWheel1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterFlyWheel2.setDirection(DcMotorSimple.Direction.FORWARD);


        waitForStart();

        if (opModeIsActive()) {
            while (opModeIsActive()) {
                double vertical = -gamepad1.left_stick_y;
                double horizontal = gamepad1.left_stick_x;
                double pivot = gamepad1.right_stick_x;

                double rightFrontSpeed = ((vertical - horizontal) - pivot);
                double rightRearSpeed = ((vertical + horizontal) - pivot);
                double leftFrontSpeed = (vertical + horizontal + pivot);
                double leftRearSpeed = ((vertical - horizontal) + pivot);

                if (gamepad1.dpad_up) {
                    powerMult = 0.5; // Change this if you want to limit maximum power of drivetrain
                } else {
                    powerMult = 1; // Back to default value
                }

                leftFront.setPower(leftFrontSpeed * powerMult);
                rightFront.setPower(rightFrontSpeed * powerMult);
                leftRear.setPower(leftRearSpeed * powerMult);
                rightRear.setPower(rightRearSpeed * powerMult);

                if (gamepad1.left_trigger > 0) {
                    intake.setPower(-1);
                } else if (gamepad1.right_trigger > 0) {
                    intake.setPower(1);
                } else {
                    intake.setPower(0);
                }

                double shoot = gamepad2.right_trigger;

                /* Use these buttons to set a constant power if you want */
                if (gamepad2.b) {
                    powerLim = 0.6;
                } else {
                    powerLim = 1;
                }

                /* A recommendation I give is using triggers for turret control as well */
                // You can put ur shooter.setPower() in there
                if (gamepad2.right_trigger > 0) {
                    shooterFlyWheel1.setPower(shoot * powerLim);
                    shooterFlyWheel2.setPower(shoot * powerLim);
                } else {
                    shooterFlyWheel1.setPower(0);
                    shooterFlyWheel2.setPower(0);
                }

            }
        }
    }
}