package org.firstinspires.ftc.teamcode.Auton.Red;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Auton.Logics.FlywheelLogic;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
@Configurable
public class OutRed extends OpMode{
    private TelemetryManager panelsTelemetry;
    private Follower follower;
    public DcMotorEx intake;
    public FlywheelLogic shooter = new FlywheelLogic();
    private Timer pathTimer, opModeTimer;
    private boolean shotsTriggered = false;
    public RobotHardware hardware;

    private enum PathState{
        Shoot,
        Out,
    }
    PathState pathState;
    public PathChain out;
    public void buildPaths() {
        out = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(87.487, 7.744),

                                new Pose(107.277, 8.650)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(90))

                .build();
    }
    public void statePathUpdate(){
        switch (pathState) {
            case Shoot:
                shooter.setTargetRPM(8000);

                if (shooter.atSpeed() && !shotsTriggered) {
                    intake.setPower(1);
                    shotsTriggered = true;
                    pathTimer.resetTimer();
                }

                if (shotsTriggered && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    intake.setPower(0);
                    shooter.stop();
                    setPathState(PathState.Out);
                }
                break;
            case Out:
                follower.followPath(out);
                break;
        }

    }

    @Override
    public void init(){
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        pathState = PathState.Shoot;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        shooter.init(hardwareMap);
        buildPaths();
    }

    private void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();

        shotsTriggered = false;
    }
    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }
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
