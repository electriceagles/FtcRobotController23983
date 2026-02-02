package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

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

    public SampleMecanumDrive drive;

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
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setDirection(DcMotorSimple.Direction.FORWARD);

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

        if (!Globals.USING_ODOMETRY) {
            resetEnc();
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
