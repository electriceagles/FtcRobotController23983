package org.firstinspires.ftc.teamcode.superTele.Testing.superAuton;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Auton.Logics.FlywheelLogic;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@Configurable
@TeleOp
public class thBallPPblue extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    //shooter setup

    private ShooterLogic shooter = new ShooterLogic();
    private boolean shotsTriggered = false;

    public enum PathState {
        //start_end pos
        //drive = movement
        //shoot = shoot

        DRIVE_STARTPOS_SHOOT_POS,

        SHOOT_PRELOAD
    }

    PathState pathState;

//    private final Pose startPose = new Pose (20.22085889570552, 122.62576687116562, Math.toRadians(144));
    private final Pose shootPose = new Pose(70.23312883435584, 78.40490797546012, Math.toRadians(135));

    private final Pose startPose = new Pose (20.22085889570552, 122.62576687116562, Math.toRadians(144));

    private PathChain driveStart2Shoot;

    public void buildPaths() {
        driveStart2Shoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
    }
    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStart2Shoot, true);
                setPathState(PathState.SHOOT_PRELOAD);
                break;

            case SHOOT_PRELOAD:
                if (!follower.isBusy()){
                    //requested shots?
                    if(!shotsTriggered) {
                        shooter.fireShots(3);
                        shotsTriggered = true;
                    }
                }
                break;

            default:
                telemetry.addLine("no state commanded");
        }
    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();

        shotsTriggered = false;
    }

    @Override
    public void init (){
        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        opModeTimer.resetTimer();
        follower = Constants.createFollower(hardwareMap);
        //TODO add in any other init mechanisms

        shooter.init(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop(){
        follower.update();
        shooter.update();
        statePathUpdate();

    }
}
