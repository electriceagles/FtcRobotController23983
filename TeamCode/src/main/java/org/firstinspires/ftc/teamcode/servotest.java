package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "testing servos", group = "TeleOp")
public class servotest extends LinearOpMode {
    public Servo gate;
    public CRServo gekko;


    @Override public void runOpMode() {
        gate = hardwareMap.get(Servo.class, "gate");
        gekko = hardwareMap.get(CRServo.class, "gekko");

        gate.setPosition(0);

        waitForStart();

        //SERVO SYSTEM MUST BE TUNED LATER TO RIGHT POSITIONS

        while (opModeIsActive()) {

            if (gamepad1.right_bumper) {
                gate.setPosition(-0.5); // moves home
                gekko.setPower(-1);
            } else if (gamepad1.left_bumper) {
                gate.setPosition(0); // moves back to gate
                gekko.setPower(0);
            }
        }

    }
}
