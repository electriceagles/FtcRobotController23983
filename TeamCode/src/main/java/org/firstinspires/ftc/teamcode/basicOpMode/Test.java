package org.firstinspires.ftc.teamcode.basicOpMode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name = "Duo TeleOp", group = "TeleOp")
public class Test extends LinearOpMode {

    private DcMotorEx shooterFlyWheel1;
    private DcMotorEx shooterFlyWheel2;

    private double powerLim = 1.0;

    @Override
    public void runOpMode() throws InterruptedException {

        // Hardware map (names MUST match Robot Configuration)
        shooterFlyWheel1 = hardwareMap.get(DcMotorEx.class, "sf1");
        shooterFlyWheel2 = hardwareMap.get(DcMotorEx.class, "sf2");

        // Motor directions (adjust if spinning wrong way)
        shooterFlyWheel1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterFlyWheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while (opModeIsActive()) {

            // Hold B to limit shooter power
            powerLim = gamepad2.b ? 0.6 : 1.0;

            // Right trigger controls shooter (0.0 → 1.0)
            double shootPower = gamepad2.right_trigger * powerLim;

            shooterFlyWheel1.setPower(shootPower);
            shooterFlyWheel2.setPower(shootPower);

            // Debug telemetry (optional)
            telemetry.addData("Trigger", gamepad2.right_trigger);
            telemetry.addData("Power Limit", powerLim);
            telemetry.addData("Shooter Power", shootPower);
            telemetry.update();
        }
    }
}