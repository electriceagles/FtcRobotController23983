package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

/**
 * AutoThreeArtifacts runs a simple autonomous routine for the BLUE alliance.
 *
 * <p>This OpMode demonstrates a basic sequence of actions for the DECODE game:
 * drive off the launch line, collect three artifacts using the intake,
 * navigate to the classifier, shoot the artifacts using the flywheels,
 * and return to the base zone.</p>
 *
 * <p><b>Tuning:</b> All timing and power values are exposed as public static
 * fields with {@code @Config} so they can be adjusted via FTC Dashboard
 * without redeploying code. Connect to http://192.168.43.1:8080/dash</p>
 *
 * <p><b>Safety:</b> The routine automatically stops before 30 seconds to
 * avoid penalties from the DECODE manual.</p>
 *
 * <p>This is the BLUE alliance version. For RED alliance, use
 * {@link AutoThreeArtifactsRed} which mirrors the strafe directions.</p>
 */
@Config
@Autonomous(name = "Auto 3 Artifacts BLUE", group = "Auto")
public class AutoThreeArtifacts extends BaseAutoOpMode {

    // ========================================================================
    // TUNABLE PARAMETERS (adjustable via FTC Dashboard)
    // ========================================================================

    // Step 1: Leave launch line
    public static double LEAVE_LINE_POWER = 0.5;
    public static long LEAVE_LINE_MS = 800;

    // Step 2: Collect artifacts
    public static double INTAKE_POWER = 1.0;
    public static long INTAKE_MS = 2000;
    public static double INTAKE_DRIVE_POWER = 0.2;  // Slow forward while intaking

    // Step 3: Navigate to classifier
    public static double STRAFE_POWER = 0.5;
    public static long STRAFE_MS = 900;
    public static double APPROACH_POWER = 0.4;
    public static long APPROACH_MS = 700;

    // Step 4: Deposit artifacts
    public static double SHOOTER_POWER = 1.0;
    public static double FEED_POWER = -1.0;  // Negative to push artifacts into shooter
    public static long DEPOSIT_MS = 3000;

    // Step 5: Return to base
    public static double RETURN_POWER = 0.5;
    public static long RETURN_MS = 1200;

    // Alliance-specific: Blue strafes RIGHT to reach classifier
    private static final int STRAFE_DIRECTION = 1;  // 1 = right, -1 = left

    @Override
    public void runOpMode() {
        // Initialize hardware using the team's existing configuration
        initHardware();

        // Set up dashboard telemetry if available
        try {
            FtcDashboard dashboard = FtcDashboard.getInstance();
            telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        } catch (Exception e) {
            // Dashboard library not present; continue with default telemetry
        }

        telemetry.addLine("=== AUTO 3 ARTIFACTS BLUE ===");
        telemetry.addLine("Status: Initialized");
        telemetry.addLine("Waiting for START...");
        telemetry.update();

        waitForStart();
        runtime.reset();

        if (isStopRequested()) return;

        // ====================================================================
        // AUTONOMOUS SEQUENCE
        // ====================================================================

        // Step 1: Drive off the launch line (3 points)
        telemetry.addLine(">>> Step 1: Leaving launch line");
        telemetry.addData("Time", "%.1f sec", runtime.seconds());
        telemetry.update();
        moveForTime(LEAVE_LINE_POWER, LEAVE_LINE_POWER, LEAVE_LINE_MS);

        if (shouldStop()) { stopAllMotors(); return; }

        // Step 2: Activate intake to collect three artifacts
        // Drive slowly forward while intaking to gather artifacts
        telemetry.addLine(">>> Step 2: Collecting artifacts");
        telemetry.addData("Time", "%.1f sec", runtime.seconds());
        telemetry.update();

        robot.intake.setPower(INTAKE_POWER);
        moveForTime(INTAKE_DRIVE_POWER, INTAKE_DRIVE_POWER, INTAKE_MS);
        robot.intake.setPower(0);

        if (shouldStop()) { stopAllMotors(); return; }

        // Step 3: Navigate to the classifier
        // Blue alliance: strafe RIGHT then drive forward
        telemetry.addLine(">>> Step 3: Navigating to classifier");
        telemetry.addData("Time", "%.1f sec", runtime.seconds());
        telemetry.update();

        strafeForTime(STRAFE_POWER, STRAFE_MS, STRAFE_DIRECTION);

        if (shouldStop()) { stopAllMotors(); return; }

        moveForTime(APPROACH_POWER, APPROACH_POWER, APPROACH_MS);

        if (shouldStop()) { stopAllMotors(); return; }

        // Step 4: Deposit the artifacts using shooter flywheels
        // The shooter launches artifacts into the classifier
        telemetry.addLine(">>> Step 4: Depositing artifacts");
        telemetry.addData("Time", "%.1f sec", runtime.seconds());
        telemetry.update();

        depositArtifacts(SHOOTER_POWER, FEED_POWER, DEPOSIT_MS);

        if (shouldStop()) { stopAllMotors(); return; }

        // Step 5: Return to the base zone (5-10 points)
        telemetry.addLine(">>> Step 5: Returning to base");
        telemetry.addData("Time", "%.1f sec", runtime.seconds());
        telemetry.update();

        moveForTime(-RETURN_POWER, -RETURN_POWER, RETURN_MS);

        // ====================================================================
        // COMPLETE
        // ====================================================================

        stopAllMotors();

        telemetry.addLine("=== AUTONOMOUS COMPLETE ===");
        telemetry.addData("Total Time", "%.1f sec", runtime.seconds());
        telemetry.update();

        // Keep telemetry visible until OpMode ends
        while (opModeIsActive() && !isStopRequested()) {
            sleep(100);
        }
    }
}
