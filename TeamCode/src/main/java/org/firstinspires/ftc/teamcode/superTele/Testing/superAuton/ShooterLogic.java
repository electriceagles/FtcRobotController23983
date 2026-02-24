package org.firstinspires.ftc.teamcode.superTele.Testing.superAuton;

import android.health.connect.datatypes.units.Velocity;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class ShooterLogic {
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private DcMotorEx intake;
    private Servo gate;
    private CRServo backServo;

    private ElapsedTime stateTimer = new ElapsedTime();

    private enum FlywheelState {
        IDLE,
        SPIN_UP, //rev shooter
        LAUNCH, //intake
        RESET_GATE //gate back
    }

    private FlywheelState flywheelState;

    //------------ Gate Constants -----------
    private double GATE_CLOSE_ANGLE = 0; //tune
    private double GATE_OPEN_ANGLE = 90; //tune
    private double GATE_OPEN_TIME = 0.4;
    private double GATE_CLOSE_TIME = 0.4; //time it takes for gate to reset


    //------------ Flywheel Constants -------------
    private int shotsRemaining = 0;
    private double flywheelVelocity = 0;
    private double MIN_FLYWHEEL_RPM = 1175;
    private double TARGET_FLYWHEEL_RPM = 1200;
    private double FLYWHEEL_MAX_SPINUP_TIME = 3.5;

    public void init (HardwareMap hwMap) {
        gate = hwMap.get(Servo.class, "gate");
        backServo = hwMap.get(CRServo.class, "back");
        shooter1 = hwMap.get(DcMotorEx.class, "sf1");
        shooter2 = hwMap.get(DcMotorEx.class, "sf2");

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = hwMap.get(DcMotorEx.class, "i");

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(111.40100, 0, 0, 14.05);

        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

        flywheelState = FlywheelState.IDLE;

        shooter1.setPower(0);
        shooter2.setPower(0);

        gate.setPosition(GATE_CLOSE_ANGLE);
    }

    public void update(){
        switch (flywheelState){
            case IDLE:
                if (shotsRemaining > 0) {
                    gate.setPosition(GATE_CLOSE_ANGLE);

                    stateTimer.reset();

                    flywheelState = FlywheelState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                shooter1.setVelocity(TARGET_FLYWHEEL_RPM);
                shooter2.setVelocity(TARGET_FLYWHEEL_RPM);

                if (stateTimer.seconds() > FLYWHEEL_MAX_SPINUP_TIME) {
                    gate.setPosition(GATE_OPEN_ANGLE);

                    stateTimer.reset();

                    flywheelState = FlywheelState.LAUNCH;
                }
                break;

            case LAUNCH:
                if (stateTimer.seconds() > GATE_OPEN_TIME) {
                    intake.setPower(-1);
                    backServo.setPower(1);
                    shotsRemaining--;

                    gate.setPosition(GATE_CLOSE_ANGLE);
                    stateTimer.reset();

                    flywheelState = FlywheelState.RESET_GATE;
                }
                break;

            case RESET_GATE:
                if (stateTimer.seconds() > GATE_CLOSE_TIME) {
                    if (shotsRemaining > 0) {
                        stateTimer.reset();
                        flywheelState = FlywheelState.SPIN_UP;
                    } else {
                        shooter1.setPower(0);
                        shooter2.setPower(0);

                        intake.setPower(0);
                        backServo.setPower(0);

                        flywheelState = FlywheelState.IDLE;
                    }
                }

                break;
        }
    }

    public void fireShots (int numberOfShots) {
        if (flywheelState == FlywheelState.IDLE) {
            shotsRemaining = numberOfShots;
        }
    }

    public boolean isBusy () {
        return flywheelState != FlywheelState.IDLE;
    }
}
