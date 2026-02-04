package org.firstinspires.ftc.teamcode.superTele.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

//So you can tune in Dashboard or Panels
@Configurable
@Config
@TeleOp
public class PositionTuner extends OpMode {

    private PIDFController controller;
    private PIDFCoefficients coefficients;
    private DcMotorEx motor;

    public static double p = 0, i = 0, d = 0, f = 0;
    public static double target;

    @Override
    public void init() {

        coefficients = new PIDFCoefficients(p, i, d, f);
        controller = new PIDFController(coefficients);

        motor = hardwareMap.get(DcMotorEx.class, "turret");
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    @Override
    public void loop() {

        /*
        These are being constantly updated
        but you can put this in your teleOps init function once you have your values
        */
        coefficients.setCoefficients(p,i,d,f);
        controller.setCoefficients(coefficients);

        //I lowkey forgot which way it was supposed to go so if you see weird behavior switch this before anything else
        double error = target - motor.getCurrentPosition();
        /* Important for the controller to know what has changed */
        controller.updateError(error);

        motor.setPower(controller.run());

        telemetry.addData("Motor Position", motor.getCurrentPosition());
        telemetry.addData("Motor Current", motor.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Motor Power", motor.getPower());
        telemetry.update();
    }
}
