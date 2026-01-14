package org.firstinspires.ftc.teamcode.basicOpMode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Q2 TeleOp", group = "TeleOp")
public class Q2TeleOpDraft extends LinearOpMode {

    public DcMotorEx lf;
    public DcMotorEx lr;
    public DcMotorEx rf;
    public DcMotorEx rr;

    public double lastSeenTagTime;

    public DcMotorEx intake;
    public Servo servo;
    public DcMotorEx shooter1;
    public DcMotorEx shooter2;
    public DcMotor turret;

    public double powerMult = 0.9;

    public VisionPortal visionPortal;
    public AprilTagProcessor aprilTag;
    public static final int at = 24; // 24 for red?

    public static final double SCAN_POWER = 0.8;
    public static final double SCAN_FREQ  = 0.25;

    public boolean turretAuto = true;
    public boolean manControl = false;
    public double scanStartTime;

    public double s_targetRPM = 4200; //tune later


    public double kP = 0.0005; //proportional gain; kP is how much to change based on current error; uses this value to add power
    public double kI = 0.0; //not using bc can lead to tm power apparently
    public double kD = 0.00012; //kD is how much to slow down (brake) if motor goes above desired rpm


    private double shooterIntegral = 0; //total error over time; initialized to be 0
    private double shooterLastError = 0; //error from last time; used to calc the rate of error


    private int lastShooterTicks = 0; //initializes the initial position/tick of motors to be 0
    private double lastShooterTime = 0; //runtime last time it shot; initialized to be 0



    private static final double TICKS_PER_REV = 28.0; //28ppr for 6k??????

    // basically if it detects a tag, it checks if it's the one we need
    private AprilTagDetection getTargetTag(List<AprilTagDetection> detections) { //lists all detections and checks, this the target tag function we use later
        if (detections == null) return null;

        for (AprilTagDetection tag : detections) { //checks each; for loop
            if (tag.id == at) {
                return tag; // stops loop
            }
        }
        return null; //only if detection didnt detect nun return null
    }

    @Override
    public void runOpMode() {

        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lr = hardwareMap.get(DcMotorEx.class, "lr");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rr = hardwareMap.get(DcMotorEx.class, "rr");

        lf.setDirection(DcMotorSimple.Direction.FORWARD);
        lr.setDirection(DcMotorSimple.Direction.FORWARD);
        rf.setDirection(DcMotorSimple.Direction.REVERSE);
        rr.setDirection(DcMotorSimple.Direction.REVERSE);

        intake = hardwareMap.get(DcMotorEx.class, "i");
        servo = hardwareMap.get(Servo.class, "servo");
        shooter1 = hardwareMap.get(DcMotorEx.class, "sf1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "sf2");

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        turret = hardwareMap.get(DcMotor.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        aprilTag = new AprilTagProcessor.Builder().build();
        WebcamName webcam = hardwareMap.get(WebcamName.class, "webcam");

        visionPortal = new VisionPortal.Builder()
                .setCamera(webcam)
                .setCameraResolution(new Size(640, 480))
                .enableLiveView(true)
                .addProcessor(aprilTag)
                .build();


        shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        scanStartTime = getRuntime();
        lastSeenTagTime = getRuntime();


        lastShooterTime = getRuntime(); //initializes rpm timing after start so no discrepancy
        lastShooterTicks = shooter1.getCurrentPosition();

        while (opModeIsActive()) {

            // controller 1 drives the bot
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            // deadzone calc?
            if (Math.abs(y)  < 0.05) y  = 0;
            if (Math.abs(x)  < 0.05) x  = 0;
            if (Math.abs(rx) < 0.05) rx = 0;

            double lfP = y + x + rx;
            double rfP = y - x - rx;
            double lrP = y - x + rx;
            double rrP = y + x - rx;

            lf.setPower(lfP * powerMult);
            rf.setPower(rfP * powerMult);
            lr.setPower(lrP * powerMult);
            rr.setPower(rrP * powerMult);

            //power limiting w button b

            if (gamepad1.right_bumper) {


                double currentRPM = getShooterRPM(); //measures current rpm at the moment the button is pressed


                double pidPower = shooterPID(currentRPM); //applies pid calculations to alter shooter power given the current rpm

                // power after being altered thru pid is used on shooter
                shooter1.setPower(pidPower);
                shooter2.setPower(pidPower);

                // telemetry for reference for tuning ig
                telemetry.addData("Target RPM", s_targetRPM);
                telemetry.addData("Current RPM", currentRPM);
                telemetry.addData("Shooter Power", pidPower);
                telemetry.update();

            } else {

                // shooter off, and pid and evt resets to 0
                shooter1.setPower(0);
                shooter2.setPower(0);

                shooterIntegral = 0;
                shooterLastError = 0;


                lastShooterTime = getRuntime();
                lastShooterTicks = shooter1.getCurrentPosition();
            }

            // controller 1 intake
            if (gamepad1.left_trigger > 0.1) {
                intake.setPower(1);
            } else if (gamepad1.left_bumper) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            // servo system
            if (gamepad1.right_trigger > 0.1) {
                servo.setPosition(gamepad1.right_trigger);
            } else {
                servo.setPosition(0);
            }

            // turret manual override
            if (gamepad1.cross) {
                turretAuto = false;
                manControl = true;
            } else if (gamepad1.triangle){
                turretAuto = true;
                manControl = false;
            }

            if (manControl) {
                if (gamepad1.dpad_left) {
                    turret.setPower(-0.4);
                } else if (gamepad1.dpad_right) {
                    turret.setPower(0.4);
                } else {
                    turret.setPower(0);
                }
            }

            // automatic turret scan
            if (turretAuto) {
                List<AprilTagDetection> detections = aprilTag.getDetections();
                AprilTagDetection targetTag = getTargetTag(detections);

                if (targetTag != null) {
                    lastSeenTagTime = getRuntime();
                }

                if (getRuntime() - lastSeenTagTime < 0.3) {
                    turret.setPower(0);
                } else {
                    double t = getRuntime() - scanStartTime;
                    double s = Math.sin(2 * Math.PI * SCAN_FREQ * t);
                    turret.setPower(SCAN_POWER * Math.signum(s));
                }
            }
        }

        turret.setPower(0);
        if (visionPortal != null) visionPortal.close();
    }


    private double getShooterRPM() {
        int currentTicks = shooter1.getCurrentPosition();
        double currentTime = getRuntime();

        double deltaTicks = currentTicks - lastShooterTicks; //how much motor moved
        double deltaTime = currentTime - lastShooterTime; //change in time/how long the motor moved for; used to find rpm

        // avoid division by 0
        if (deltaTime <= 0) return 0;

        // calculate rpm
        double revolutions = deltaTicks / TICKS_PER_REV;
        double revPerSec = revolutions / deltaTime;
        double rpm = revPerSec * 60.0;

        // Save state for next call
        lastShooterTicks = currentTicks;
        lastShooterTime = currentTime;

        // rpm can be negative and we dont want that nuh uh no no no
        return Math.abs(rpm);
    }


    //also:
    //P term: pushes harder the farther we are from target rpm
    //D term: reacts to the change in error (damps overshoot / oscillation)

    //actually applying pid for shooter
    private double shooterPID(double currentRPM) {

        //our error
        double error = s_targetRPM - currentRPM;


        //accumulation of error over time (net change)
        shooterIntegral += error;

        //looks at how quickly the error is changing and helps brake so its not overpowered
        double derivative = error - shooterLastError;

        //save error for next calculation
        shooterLastError = error;

        // Combine PID terms into one value; formula: output = kP + kI + kD
        double output = (kP * error) + (kI * shooterIntegral) + (kD * derivative);

        //max motor power is 1
        output = Math.max(0.0, Math.min(1.0, output));

        return output; //makes shooterPID the value after getting all those numbers tg
    }
}