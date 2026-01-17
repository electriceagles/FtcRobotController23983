package org.firstinspires.ftc.teamcode.basicOpMode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class FlywheelLogic {

    private DcMotorEx shooterFlyWheel1;
    private DcMotorEx shooterFlyWheel2;
    private Servo servo;

    private DcMotorEx turret;

    private ElapsedTime stateTimer = new ElapsedTime();

    private enum FlywheelState {
        IDLE,
        SPIN_UP,
        OPEN,
        CLOSE

    }
    private int lastShooterTicks = 0;
    private double lastShooterTime = 0;
    private static final double TICKS_PER_REV = 28.0;
    private FlywheelState flywheelState;

    //SERVO GATE CONSTANTS :)
    private double CLOSE_ANGLE = 0;
    private double OPEN_ANGLE = 90;
    private double OPEN_TIME = 0.5;
    private double CLOSE_TIME = 0.5;

    //FLYWHEEL CONSTANTS ;)
    private int shotsRemaining = 0;
    private double flywheelVelocity = 0;
    private double min_Flywheel_RPM = 4100; //tune later
    public double s_targetRPM = 4200; //tune later
    private double FLYWHEEL_SPINUP_TIME = 2;
    private double FLYWHEEL_SHOOT_TIME = 3;

    public void init(HardwareMap hardwareMap) {
        shooterFlyWheel1 = hardwareMap.get(DcMotorEx.class, "sf1");
        shooterFlyWheel2 = hardwareMap.get(DcMotorEx.class, "sf2");
        servo = hardwareMap.get(Servo.class, "servo");

        turret = hardwareMap.get(DcMotorEx.class, "turret");
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterFlyWheel1.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterFlyWheel2.setDirection(DcMotorSimple.Direction.REVERSE);

        shooterFlyWheel1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterFlyWheel1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterFlyWheel2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterFlyWheel2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        flywheelState = FlywheelState.IDLE;
        shooterFlyWheel1.setPower(0);
        shooterFlyWheel2.setPower(0);

        servo.setPosition(CLOSE_ANGLE);

    }

    public void update() {
        switch (flywheelState) {
            case IDLE:
                if (shotsRemaining > 0) {
                    servo.setPosition(CLOSE_ANGLE);
                    shooterFlyWheel1.setPower(s_targetRPM);
                    shooterFlyWheel2.setPower(s_targetRPM);

                    stateTimer.reset();
                    flywheelState = FlywheelState.SPIN_UP;

                }
                break;
            case SPIN_UP:
                if (flywheelVelocity > min_Flywheel_RPM || stateTimer.seconds() > FLYWHEEL_SPINUP_TIME) {
                    servo.setPosition(OPEN_ANGLE);
                    stateTimer.reset();
                    flywheelState = FlywheelState.OPEN;
            }
                break;
            case OPEN:
                if (stateTimer.seconds() > OPEN_TIME) {
                     shotsRemaining--;
                     servo.setPosition(CLOSE_ANGLE);
                     stateTimer.reset();

                     flywheelState = FlywheelState.CLOSE;
                }
                break;
            case CLOSE:
                if (stateTimer.seconds() > CLOSE_TIME) {
                    if (shotsRemaining > 0) {
                        stateTimer.reset();
                        flywheelState = FlywheelState.SPIN_UP;
                    } else {
                        shooterFlyWheel1.setPower(0);
                        shooterFlyWheel2.setPower(0);
                        flywheelState = FlywheelState.IDLE;
                    }
                }
                break;
        }


    }
    public void fireShots(int numberofShots) {
        if (flywheelState == FlywheelState.IDLE){
            shotsRemaining = numberofShots;
        }
    }
    public boolean isBusy(){
        return flywheelState != FlywheelState.IDLE;
    }
}
