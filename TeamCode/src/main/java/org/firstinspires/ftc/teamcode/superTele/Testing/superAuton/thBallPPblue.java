package org.firstinspires.ftc.teamcode.superTele.Testing.superAuton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp
public class thBallPPblue extends OpMode {

    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {
        //start_end pos
        //drive = movement
        //shoot = shoot

        DRIVE_STARTPOS_SHOOT_POS,

        SHOOT_PRELOAD
    }

    PathState pathState;

    private final Pose startPose = new Pose (20.22085889570552, 122.62576687116562, Math.toRadians(144));
    private final Pose shootPose = new Pose(70.23312883435584, 78.40490797546012, Math.toRadians(135));

    private PathChain driveStart2Shoot;

    public void buildPaths() {
//        driveStart2Shoot = new PathChain(
//                new Path(new BezierLine(startPose, shootPose))
//        );

        driveStart2Shoot = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();

    }

    @Override
    public void init (){

    }

    @Override
    public void loop(){

    }
}
