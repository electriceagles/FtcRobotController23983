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
public class AutonGate extends OpMode {
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


    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                /* Score Preload */
                follower.followPath(scorePreload);
                //shoot:
                if (follower.getPose().getX() > 54 && follower.getPose().getY() > 102) {
                    stateTimer.reset();
                    shooter1.setVelocity(shooterRPM);
                    shooter2.setVelocity(shooterRPM);
                }

                if (stateTimer.seconds() > revTime) {
                    intake.setPower(-1);
                }

                if (stateTimer.seconds() > 6) {
                    shooter1.setVelocity(0);
                    shooter2.setVelocity(0);
                    setPathState(1);
                }
                break;

            case 1:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(Pickup1setup, true);
                    //intake

                    if (follower.getPose().getX() > 38 && follower.getPose().getY() > 81) {
                        intake.setPower(-1);
                        setPathState(2);
                    }

                }
                break;


            case 2:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1, true);

                    if (follower.getPose().getX() > 14 && follower.getPose().getY() > 82) {
                        intake.setPower(0);
                        setPathState(3);
                    }
                }
                break;

            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if (!follower.isBusy()) {
                    //shoot
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1, true);

                    //shoot:
                    if (follower.getPose().getX() > 54 && follower.getPose().getY() > 102) {
                        stateTimer.reset();
                        shooter1.setVelocity(shooterRPM);
                        shooter2.setVelocity(shooterRPM);
                    }

                    if (stateTimer.seconds() > revTime) {
                        intake.setPower(-1);
                    }

                    if (stateTimer.seconds() > 6) {
                        shooter1.setVelocity(0);
                        shooter2.setVelocity(0);
                        setPathState(4);

                    }
                }
                break;

            case 4:
            /* You could check for
            - Follower State: "if(!follower.isBusy()) {}"
            - Time: "if(pathTimer.getElapsedTimeSeconds() > 1) {}"
            - Robot Position: "if(follower.getPose().getX() > 36) {}"
            */
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(gateSetup, true);
                    //intake
                    setPathState(5);
                }
                break;

            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(openGate, true);
                    setPathState(6);
                }
                break;

            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    //"park"
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(homePose, true);
                    setPathState(7);
                }
                break;

            case 7:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if (!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }

    /**
     * These change the states of the paths and actions. It will also reset the timers of the individual switches
     **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot, these must be called continuously in order to work
        follower.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        stateTimer.reset();


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

        shooter1 = hardwareMap.get(DcMotorEx.class, "sf1");
        shooter2 = hardwareMap.get(DcMotorEx.class, "sf2");

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = hardwareMap.get(DcMotorEx.class, "i");

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(111.40100, 0, 0, 14.05);

        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);

    }


    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {}




}
