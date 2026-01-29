package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

/**
 * I made this file so you guys can store all your hardware items / initialization in here
 * this makes it easier when you write other files as you can call the hardware from here
 */
@Config
public class RobotHardware {

    public DcMotorEx leftFront;
    public DcMotorEx rightRear;
    public DcMotorEx leftRear;
    public DcMotorEx rightFront;

    public DcMotorEx intake;

    public DcMotorEx shooterFlyWheel1;
    public DcMotorEx shooterFlyWheel2;
    public Servo servo;
    public Servo hood;
    public DcMotorEx turret;

    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTag;

    /* This drive file should only be used for auto */
    public SampleMecanumDrive drive;

    public double powerMult = 0.7;
    public double shooterLimit = 1;

    /* Whenever you guys want to initialize your hardware call this function  */
    public void init(HardwareMap hardwareMap, boolean isAuto) {

        if (isAuto) {
            drive = new SampleMecanumDrive(hardwareMap);
        }

        // DRIVETRAIN
        leftFront = hardwareMap.get(DcMotorEx.class, "lf");
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftRear = hardwareMap.get(DcMotorEx.class, "lr");
        leftRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightFront = hardwareMap.get(DcMotorEx.class, "rf");
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightRear = hardwareMap.get(DcMotorEx.class, "rr");
        rightRear.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // INTAKE + SHOOTER + TURRET
        intake = hardwareMap.get(DcMotorEx.class, "i");
        shooterFlyWheel1 = hardwareMap.get(DcMotorEx.class, "sf1");
        shooterFlyWheel2 = hardwareMap.get(DcMotorEx.class, "sf2");

        hood = hardwareMap.get(Servo.class, "hood");
        servo = hardwareMap.get(Servo.class, "servo");

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // TODO: Please test directions and then reverse if needed
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftRear.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightRear.setDirection(DcMotor.Direction.FORWARD);

        intake.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterFlyWheel1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterFlyWheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterFlyWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterFlyWheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterFlyWheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterFlyWheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        turret.setDirection(DcMotorSimple.Direction.FORWARD);

        if (!Globals.USING_ODOMETRY) {
            resetEnc();
        }
    }

    /* This function exists so you don't have to write a whole teleOp everytime u change something */
    public void duoTeleOp(Gamepad gamepad1, Gamepad gamepad2) {

        /* === DRIVETRAIN CONTROL ===  CONTROLLER 1 */
        double vertical = -gamepad1.left_stick_y;
        double horizontal = gamepad1.left_stick_x;
        double pivot = gamepad1.right_stick_x;

        double rightFrontSpeed = ((vertical - horizontal) - pivot);
        double rightRearSpeed = ((vertical + horizontal) - pivot);
        double leftFrontSpeed = (vertical + horizontal + pivot);
        double leftRearSpeed = ((vertical - horizontal) + pivot);

        if (gamepad1.dpad_up) { // TODO: Change this button for the driver
            powerMult = 0.5; // Change this if you want to limit maximum power of drivetrain
        } else {
            powerMult = 1;
        }

        leftFront.setPower(leftFrontSpeed * powerMult);
        rightFront.setPower(rightFrontSpeed * powerMult);
        leftRear.setPower(leftRearSpeed * powerMult);
        rightRear.setPower(rightRearSpeed * powerMult);

        /* === INTAKE CONTROL ===  CONTROLLER 1 */
        if (gamepad1.right_trigger > 0.1) {
            intake.setPower(1);
        } else if (gamepad1.left_trigger > 0.1) {
            intake.setPower(-1);
        } else {
            intake.setPower(0);
        }

        /* === SHOOTER CONTROL ===  CONTROLLER 2 */
        shooterLimit = gamepad2.b ? 0.6 : 1.0;

        if (gamepad2.right_trigger > 0.05) {
            shooterFlyWheel1.setPower(gamepad2.right_trigger * shooterLimit);
            shooterFlyWheel2.setPower(gamepad1.right_trigger * shooterLimit);
        } else {
            shooterFlyWheel1.setPower(0);
            shooterFlyWheel2.setPower(0);
        }

        /* === TURRET CONTROL ===  CONTROLLER 2 */
        if (Math.abs(gamepad2.right_stick_x) > 0.1) {
            turret.setPower(gamepad2.left_stick_x);
        }

    }

    // Method to reset drivetrain encoders ONLY
    public void resetEnc() {
        leftRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightRear.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

}
