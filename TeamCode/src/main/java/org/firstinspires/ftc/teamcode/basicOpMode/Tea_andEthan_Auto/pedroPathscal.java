package org.firstinspires.ftc.teamcode.basicOpMode.Tea_andEthan_Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.basicOpMode.FlywheelLogic;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@TeleOp
public class pedroPathscal extends OpMode {
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    //FLYWHEEL LOGIC SETUP
    private FlywheelLogic shooter = new FlywheelLogic();
    private boolean shotsTriggered = false;

    public enum PathState{
        Drive_Start2Shoot,
        ShootPreload,

    }
    PathState pathState;
    private final Pose startPose = new Pose(59.29519450800915,7,Math.toRadians(90));
    private final Pose shootPose = new Pose(59.29519450800915,84.27459954233409,Math.toRadians(129));
    public PathChain intake1in;
    public PathChain intake1out;
    public PathChain intake2in;

    private PathChain driveStart;
    public void buildPaths(){
        //Starting,Ending
        driveStart = follower.pathBuilder()
                .addPath(new BezierLine(startPose,shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(),shootPose.getHeading())
                .build();

        intake1in = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(59.295, 84.275),
                                new Pose(19.771, 84.275)
                        )
                ).setTangentHeadingInterpolation()
                .build();

        intake1out = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(19.771, 84.275),
                                new Pose(52.373, 95.867),
                                new Pose(59.295, 84.110)
                        )
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();
        intake2in = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(59.295, 84.110),
                                new Pose(56.318, 66.189),
                                new Pose(41.098, 59.037),
                                new Pose(15.652, 59.643)
                        )
                ).setTangentHeadingInterpolation()

                .build();
    }
    public void statePathUpdate(){
        switch (pathState){
            case Drive_Start2Shoot:
                follower.followPath(driveStart, true);
                setPathState(PathState.ShootPreload );
                break;
            case ShootPreload:
                if (!follower.isBusy()){

                    //ToDo add flywheelLogic
                    if (!shotsTriggered){

                    }
                    telemetry.addLine("First Path Done :)");
                }
                break;
            default:
                telemetry.addLine("No Command :(");
                break;
        }
    }
    public void setPathState(PathState newState){
        pathState = newState;
        pathTimer.resetTimer();

        shotsTriggered = false;
    }


    @Override
    public void init(){
        pathState = PathState.Drive_Start2Shoot;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        shooter.init(hardwareMap);

        buildPaths();
        follower.setPose(startPose);

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
        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading",follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());


    }
}