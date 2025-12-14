package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="MainTeleOp_WithTurretVision", group="FINAL")
public class MainTeleOp_WithTurretVision extends LinearOpMode {

    // drive motors
    private DcMotor lf, rf, lb, rb;
    
    //other motors
    private DcMotor turet;
    private DcMotor intake;
    private DcMotor o1, o2;
    
    //calls vision modules
    private ScanStopScanModule scanModule;
    private TurretTagAimerModule turretAimer;

    private boolean turretAimMode = false;
    private boolean turretScanMode = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // --- DRIVE MOTORS ---
        lf = hardwareMap.dcMotor.get("lf");
        rf = hardwareMap.dcMotor.get("rf");
        lb = hardwareMap.dcMotor.get("lb");
        rb = hardwareMap.dcMotor.get("rb");

        // --- SUBSYSTEM MOTORS ---
        intake  = hardwareMap.dcMotor.get("intake");
        outtake1 = hardwareMap.dcMotor.get("o1");
        outtake2 = hardwareMap.dcMotor.get("o2")
        turret = hardwareMap.dcMotor.get("turret");
        
        // --- VISION MODULES ---
        scanModule = new ScanStopScanModule(hardwareMap, getRuntime());
        turretAimer = new TurretTagAimerModule(hardwareMap, getRuntime());

        waitForStart();

        while (opModeIsActive()) {

            double runtime = getRuntime();

            // ===== DRIVER CONTROLS =====
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x;
            double r = gamepad1.right_stick_x;

            double lfP = y + x + r;
            double rfP = y - x - r;
            double lbP = y - x + r;
            double rbP = y + x - r;

            lf.setPower(lfP);
            rf.setPower(rfP);
            lb.setPower(lbP);
            rb.setPower(rbP);

            // intake
            if (gamepad1.left_trigger > 0.1)
                intake.setPower(1.0);
            else if (gamepad1.left_bumper)
                intake.setPower(-1.0);
            else
                intake.setPower(0);
            
            // outtake wheel
            if (gamepad1.right_trigger > 0.1)
                outtake1.setPower(1.0)
                outtake2.setPower(-1.0)
            else
                outtake.setPower(0);
            
            // ===== TURRET VISION MODES =====
            if (gamepad1.dpad_left) {
                turretAimMode = true;
                turretScanMode = false;
            }
            if (gamepad1.dpad_right) {
                turretScanMode = true;
                turretAimMode = false;
            }
            if (gamepad1.dpad_down) {
                turretAimMode = false;
                turretScanMode = false;
            }

            if (turretAimMode) {
                turretAimer.update(runtime);
                telemetry.addLine("TURRET: AIMING MODE");
            } else if (turretScanMode) {
                scanModule.update(runtime);
                telemetry.addLine("TURRET: SCAN MODE");
            } else {
                telemetry.addLine("TURRET: MANUAL");
                turret.setPower(gamepad1.right_stick_y * 0.5);
            }

            telemetry.update();
        }

        scanModule.stop();
        turretAimer.stop();
    }
}
