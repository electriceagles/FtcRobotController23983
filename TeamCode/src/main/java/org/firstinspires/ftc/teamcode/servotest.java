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

        //SERVO SYSTEM MUST BE TUNED LATER TO RIGHT POSITIONS
        if (gamepad1.right_trigger > 0.1) {
            gate.setPosition(0.5); // moves servo 90 degrees
            gekko.setPower(-1);
        } else {
            gate.setPosition(0); // moves back to og position
            gekko.setPower(0);
        }
    }
}
