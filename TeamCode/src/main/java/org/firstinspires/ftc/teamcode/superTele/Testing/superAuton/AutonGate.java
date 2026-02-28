package org.firstinspires.ftc.teamcode.superTele.Testing.superAuton; // make sure this aligns with class location
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class AutonGate {
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private ElapsedTime stateTimer = new ElapsedTime();

    private int pathState;

    //hardware:
    private DcMotorEx shooter1;
    private DcMotorEx shooter2;
    private DcMotorEx intake;
    private Servo gate;

    private double shooterRPM = 1120; //time it takes for gate to reset
    private double revTime = 3.5; //time it takes for gate to reset


    private final Pose startPose = new Pose(56, 8, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(56, 104, Math.toRadians(140)); // Scoring Pose of our robot. It is facing the goal at a 138 degree angle.
    private final Pose pickup1set = new Pose(40.52117263843648, 83.72638436482086, Math.toRadians(180));
    private final Pose pickup1Pose = new Pose(15.244299674267102, 83.72638436482086, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose gateSet = new Pose(56.17915309446255, 72.05211726384363, Math.toRadians(180));
    private final Pose gatePose = new Pose(13.276872964169383, 71.81107491856676, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose returnPose = new Pose(56, 104, Math.toRadians(90));

    private Path scorePreload;
    private PathChain Pickup1setup, grabPickup1, scorePickup1, gateSetup, openGate, homePose;

    public void buildPaths() { //edit
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        Pickup1setup = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1set))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1set, pickup1Pose))
                .setLinearHeadingInterpolation(pickup1set.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        gateSetup = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, gateSet))
                .setLinearHeadingInterpolation(scorePose.getHeading(), gateSet.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        openGate = follower.pathBuilder()
                .addPath(new BezierLine(gateSet, gatePose))
                .setLinearHeadingInterpolation(gateSet.getHeading(), gatePose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        homePose = follower.pathBuilder()
                .addPath(new BezierLine(gatePose, returnPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), returnPose.getHeading())
                .build();

    }





}
