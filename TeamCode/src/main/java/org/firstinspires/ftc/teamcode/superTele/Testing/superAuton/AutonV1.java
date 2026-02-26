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

@Autonomous(name = "9 ball Auto", group = "Examples")
public class AutonV1 extends OpMode {
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
    private double GATE_CLOSE_ANGLE = 0; //tune
    private double GATE_OPEN_ANGLE = 90; //tune
    private double GATE_OPEN_TIME = 0.4;
    private double GATE_CLOSE_TIME = 0.4; //time it takes for gate to reset

    private final Pose startPose = new Pose(56, 8, Math.toRadians(90)); // Start Pose of our robot.
    private final Pose scorePose = new Pose(72, 72, Math.toRadians(138)); // Scoring Pose of our robot. It is facing the goal at a 138 degree angle.
    private final Pose pickup1Pose = new Pose(15.244299674267102, 83.72638436482086, Math.toRadians(180)); // Highest (First Set) of Artifacts from the Spike Mark.
    private final Pose pickup2Pose = new Pose(13.8371335504886, 59.57003257328989, Math.toRadians(180)); // Middle (Second Set) of Artifacts from the Spike Mark.
    private final Pose returnPose = new Pose(72, 72, Math.toRadians(90));

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, homePose;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, scorePose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

        /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup1Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1Pose.getHeading())
                .build();

        /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1Pose, scorePose))
                .setLinearHeadingInterpolation(pickup1Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, pickup2Pose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2Pose.getHeading())
                .build();

        /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2Pose, scorePose))
                .setLinearHeadingInterpolation(pickup2Pose.getHeading(), scorePose.getHeading())
                .build();

        /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        homePose = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, returnPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), returnPose.getHeading())
                .build();

    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                /* Score Preload */
                follower.followPath(scorePreload);
                //shoot:
                shooter1.setVelocity(shooterRPM);
                shooter2.setVelocity(shooterRPM);

                if (stateTimer.seconds() > revTime){
                    gate.setPosition(GATE_OPEN_ANGLE);
                    intake.setPower(-1);
                }

                if (stateTimer.seconds() > 6) {
                    stateTimer.reset();
                    shooter1.setVelocity(0);
                    shooter2.setVelocity(0);
                    gate.setPosition(GATE_CLOSE_ANGLE);
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
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1,true);
                    //intake
                    intake.setPower(-1);

                    if(follower.getPose().getX() > 15) {
                        intake.setPower(0);
                        setPathState(2);
                    }
                }
                break;

            case 2:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup1Pose's position */
                if(!follower.isBusy()) {
                   //shoot
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup1,true);

                    //shoot:
                    shooter1.setVelocity(shooterRPM);
                    shooter2.setVelocity(shooterRPM);

                    if (stateTimer.seconds() > revTime){
                        gate.setPosition(GATE_OPEN_ANGLE);
                        intake.setPower(-1);
                    }

                    if (stateTimer.seconds() > 6) {
                        stateTimer.reset();
                        shooter1.setVelocity(0);
                        shooter2.setVelocity(0);
                        gate.setPosition(GATE_CLOSE_ANGLE);
                        setPathState(3);

                    }
                }
                break;

            case 3:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Grab Sample */
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(grabPickup1,true);
                    //intake
                    intake.setPower(-1);

                    if(follower.getPose().getX() > 13) {
                        intake.setPower(0);
                        setPathState(4);
                    }
                }
                break;


            case 4:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the pickup2Pose's position */
                if(!follower.isBusy()) {
                   //shoot
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are scoring the sample */
                    follower.followPath(scorePickup2,true);
                    shooter1.setVelocity(shooterRPM);
                    shooter2.setVelocity(shooterRPM);

                    if (stateTimer.seconds() > revTime){
                        gate.setPosition(GATE_OPEN_ANGLE);
                        intake.setPower(-1);
                    }

                    if (stateTimer.seconds() > 6) {
                        stateTimer.reset();
                        shooter1.setVelocity(0);
                        shooter2.setVelocity(0);
                        gate.setPosition(GATE_CLOSE_ANGLE);
                        setPathState(5);
                    }
                }
                break;

            case 5:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    //"park" at 72,72
                    /* Since this is a pathChain, we can have Pedro hold the end point while we are grabbing the sample */
                    follower.followPath(homePose,true);
                    setPathState(6);
                }
                break;

            case 6:
                /* This case checks the robot's position and will wait until the robot position is close (1 inch away) from the scorePose's position */
                if(!follower.isBusy()) {
                    /* Set the state to a Case we won't use or define, so it just stops running an new paths */
                    setPathState(-1);
                }
                break;
        }
    }
    /** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
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


        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);

    }
    public void init (HardwareMap hwMap) {
        gate = hwMap.get(Servo.class, "gate");
        shooter1 = hwMap.get(DcMotorEx.class, "sf1");
        shooter2 = hwMap.get(DcMotorEx.class, "sf2");

        shooter1.setDirection(DcMotorSimple.Direction.REVERSE);
        shooter2.setDirection(DcMotorSimple.Direction.FORWARD);

        shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        intake = hwMap.get(DcMotorEx.class, "i");

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(111.40100, 0, 0, 14.05);

        shooter1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        shooter2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);


        shooter1.setPower(0);
        shooter2.setPower(0);

        gate.setPosition(GATE_CLOSE_ANGLE);
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