package org.firstinspires.ftc.teamcode.basicOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "test servo", group = "TeleOp")
public class servotest extends LinearOpMode {
    public Servo servo;

    @Override public void runOpMode() {
        servo = hardwareMap.get(Servo.class, "servo");

        //SERVO SYSTEM MUST BE TUNED LATER TO RIGHT POSITIONS
        if (gamepad1.right_trigger > 0.1) {
            servo.setPosition(0.5); // moves servo 90 degrees
        } else {
            servo.setPosition(0); // moves back to og position
        }
    }
}
