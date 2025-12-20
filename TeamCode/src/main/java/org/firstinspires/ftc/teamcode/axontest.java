package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "Test Axon")
public class axontest extends LinearOpMode {
    private Servo Axon;
    private ServoController ControlHub_ServoController;
    private boolean ChangeDirection;

    @Override
    public void runOpMode() {
        ControlHub_ServoController = hardwareMap.get(ServoController.class, "Control Hub");
        ControlHub_ServoController.pwmEnable();
        Axon = hardwareMap.get(Servo.class, "servo");
        ChangeDirection = true;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.right_trigger > 0.1) {
                if (ChangeDirection) {
                    Axon.setPosition(1.0);
                } else {
                    Axon.setPosition(0.0);
                }
                ChangeDirection = !ChangeDirection;
                sleep(500);
            }
        }
    }
}
