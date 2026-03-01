/*package org.firstinspires.ftc.teamcode.Auton.Red;

import static org.firstinspires.ftc.teamcode.pedroPathing.Tuning.follower;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Auton.Blue.pedroPathscalBlueV1;
import org.firstinspires.ftc.teamcode.Auton.Logics.FlywheelLogic;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Configurable
@Autonomous
public class FarGoodR extends OpMode {
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public DcMotorEx intake;
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    //FLYWHEEL LOGIC SETUP
    public FlywheelLogic shooter = new FlywheelLogic();
    private boolean shotsTriggered = false;
    private RobotHardware hardware;
    public enum PathState{

    }

    PathState pathState;

    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();

        shotsTriggered = false;
    }


    @Override
    public void init(){
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        pathState = PathState.Drive_Start2Shoot;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        shooter.init(hardwareMap);
        buildPaths();
    }
    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop(){
        follower.update();
        shooter.update();

        statePathUpdate();
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);


    }
}
*/