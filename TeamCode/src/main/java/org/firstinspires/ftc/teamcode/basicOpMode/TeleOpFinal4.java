package org.firstinspires.ftc.teamcode.basicOpMode;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp(name = "Q1 TeleOp FINAL", group = "TeleOp")
public class TeleOpFinal4 extends LinearOpMode {

    public DcMotorEx lf;
    public DcMotorEx lr;
    public DcMotorEx rf;
    public DcMotorEx rr;

    public DcMotorEx intake;
    public Servo servo; //axon; the one w red sticker is pre programmed, use that
    public DcMotorEx shooter1;
    public DcMotorEx shooter2;
    public DcMotor turret;

    public double powerMult = 0.7; //change this depending on how fast we want robot to be
    public double powerLim = 0.8;


    @Override
    public void runOpMode() {


        lf = hardwareMap.get(DcMotorEx.class, "lf");
        lr = hardwareMap.get(DcMotorEx.class, "lr");
        rf = hardwareMap.get(DcMotorEx.class, "rf");
        rr = hardwareMap.get(DcMotorEx.class, "rr");

        //*note to self: change the directions later once robot is finished*
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


        waitForStart();

        while (opModeIsActive()) {

            // controller 1 drives the bot
            double y = gamepad1.left_stick_y;
            double x = -gamepad1.left_stick_x;
            double rx = -gamepad1.right_stick_x;

            //deadzone to avoid stick drift (hopefully works...)
            if (Math.abs(y) < 0.05) y = 0;
            if (Math.abs(x) < 0.05) x = 0;
            if (Math.abs(rx) < 0.05) rx = 0;

            double lfP = y + x + rx;
            double rfP = y - x - rx;
            double lrP = y - x + rx;
            double rrP = y + x - rx;


            lf.setPower(lfP * powerMult);
            rf.setPower(rfP * powerMult);
            lr.setPower(lrP * powerMult);
            rr.setPower(rrP * powerMult);

            // controller 1 rev up shooter (bottom right)

            powerLim = gamepad2.circle ? 0.7 : 0.8;

            double shootPower = gamepad2.right_trigger * powerLim;

            shooter1.setPower(shootPower);
            shooter2.setPower(shootPower);

            if (gamepad1.right_bumper) {
                shooter1.setPower(powerLim);
                shooter2.setPower(powerLim);
            } else {
                shooter1.setPower(0);
                shooter2.setPower(0);
            }

            // controller 1 intake (bottom left is intake, top left is discard)

            if (gamepad1.left_trigger > 0.1) {
                intake.setPower(1);
            } else if (gamepad1.left_bumper) {
                intake.setPower(-1);
            } else {
                intake.setPower(0);
            }

            // servo system (holding top right moves it 90deg)
            //SERVO SYSTEM MUST BE TUNED LATER TO RIGHT POSITIONS
            if (gamepad1.right_bumper) {
                servo.setPosition(0.67); // moves servo 120 degrees
            } else {
                servo.setPosition(0); // moves back to og position
            }

            // code for turret

            //manual turret override
            //function for manual control of turret

            if (gamepad1.dpad_left) {
                // overriding the auto scan
                turret.setPower(-0.4); //TUNE LATER FOR DRIVER EFFICIENCY
            } else if (gamepad1.dpad_right) {
                turret.setPower(0.4); //TUNE LATER FOR DRIVER EFFICIENCY
            } else {
                turret.setPower(0);
            }


        }


    }
}