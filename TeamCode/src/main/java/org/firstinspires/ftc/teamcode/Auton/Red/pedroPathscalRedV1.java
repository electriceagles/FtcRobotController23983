package org.firstinspires.ftc.teamcode.Auton.Red;

import static android.os.SystemClock.sleep;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Auton.Logics.FlywheelLogic;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@TeleOp
public class pedroPathscalRedV1 extends OpMode {


    public DcMotorEx intake;
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    //FLYWHEEL LOGIC SETUP
    private FlywheelLogic shooter = new FlywheelLogic();
    private boolean shotsTriggered = false;
    public RobotHardware hardware;

    public enum PathState{
        Drive_Start2Shoot,
        ShootPreload,
        Shoot2,
        Shoot3,
        Shoot4,
        Intake1,
        InOutTake1,
        Intake2,
        Clear,
        InOuttake2,
        Intake3,
        InOutTake3,
        Out,


    }
    PathState pathState;
    public PathChain start;
    public PathChain intake1in;
    public PathChain intake1out;
    public PathChain intake2in;
    public PathChain clear;
    public PathChain intake2out;
    public PathChain intake3in;
    public PathChain intake3out;
    public PathChain outOfZone;
    public void buildPaths(){
        start = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(84.705, 7.000),

                                new Pose(84.705, 84.275)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(44))

                .build();

        intake1in = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(84.705, 84.275),

                                new Pose(126.700, 84.275)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        intake1out = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(126.700, 84.275),

                                new Pose(84.705, 84.110)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(44))
                .setReversed()
                .build();

        intake2in = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(84.705, 84.110),
                                new Pose(87.682, 66.212),
                                new Pose(102.902, 59.037),
                                new Pose(125.053, 59.643)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        clear = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(125.053, 59.643),

                                new Pose(127.680, 71.059)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(0))

                .build();

        intake2out = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(127.680, 71.059),

                                new Pose(84.705, 84.110)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(44))
                .setReversed()
                .build();

        intake3in = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(84.705, 84.110),
                                new Pose(85.672, 28.974),
                                new Pose(128.890, 35.719)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(44), Math.toRadians(0))

                .build();

        intake3out = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(128.890, 35.719),

                                new Pose(84.945, 84.197)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(44))

                .build();

        outOfZone = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(84.945, 84.197),

                                new Pose(96.172, 79.801)
                        )
                ).setTangentHeadingInterpolation()
                .build();
    }
    public void statePathUpdate(){
        switch (pathState) {
            case Drive_Start2Shoot:
                follower.followPath(start, true);
                setPathState(pedroPathscalRedV1.PathState.ShootPreload);
                break;
            case ShootPreload:
                shooter.setTargetRPM(4200);

                if (shooter.atSpeed() && !shotsTriggered) {
                    intake.setPower(1);
                    //hardware.servo.setPosition(0.67);
                    shotsTriggered = true;
                    pathTimer.resetTimer();
                }

                if (shotsTriggered && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    //hardware.servo.setPosition(0);
                    intake.setPower(0);
                    shooter.stop();
                    setPathState(pedroPathscalRedV1.PathState.Intake1);
                }
                break;
            case Intake1:
                intake.setPower(1);
                follower.followPath(intake1in, true);
                intake.setPower(0);
                setPathState(pedroPathscalRedV1.PathState.InOutTake1);
                break;
            case InOutTake1:
                follower.followPath(intake1out, true);
                setPathState(pedroPathscalRedV1.PathState.Shoot2);
                break;
            case Shoot2:
                shooter.setTargetRPM(4200);

                if (shooter.atSpeed() && !shotsTriggered) {
                    intake.setPower(1);
                    //hardware.servo.setPosition(0.67);
                    shotsTriggered = true;
                    pathTimer.resetTimer();
                }

                if (shotsTriggered && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    //hardware.servo.setPosition(0);
                    intake.setPower(0);
                    shooter.stop();
                    setPathState(pedroPathscalRedV1.PathState.Intake2);
                }
                break;

            case Intake2:
                intake.setPower(1);
                follower.followPath(intake2in);
                intake.setPower(0);
                setPathState(pedroPathscalRedV1.PathState.Clear);
                break;
            case Clear:
                follower.followPath(clear);
                sleep(1000);
                setPathState(pedroPathscalRedV1.PathState.InOuttake2);
                break;
            case InOuttake2:
                follower.followPath(intake2out);
                setPathState(pedroPathscalRedV1.PathState.Shoot3);
                break;
            case Shoot3:
                shooter.setTargetRPM(4200);

                if (shooter.atSpeed() && !shotsTriggered) {
                    intake.setPower(1);
                    //hardware.servo.setPosition(0.67);
                    shotsTriggered = true;
                    pathTimer.resetTimer();
                }

                if (shotsTriggered && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    //hardware.servo.setPosition(0);
                    intake.setPower(0);
                    shooter.stop();
                    setPathState(pedroPathscalRedV1.PathState.Intake3);
                }
                break;
            case Intake3:
                intake.setPower(1);
                follower.followPath(intake3in);
                intake.setPower(0);
                setPathState(pedroPathscalRedV1.PathState.InOutTake3);
                break;
            case InOutTake3:
                follower.followPath(intake3out);
                setPathState(pedroPathscalRedV1.PathState.Shoot4);
                break;
            case Shoot4:
                shooter.setTargetRPM(4200);

                if (shooter.atSpeed() && !shotsTriggered) {
                    intake.setPower(1);
                    //hardware.servo.setPosition(0.67);
                    shotsTriggered = true;
                    pathTimer.resetTimer();
                }

                if (shotsTriggered && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    //hardware.servo.setPosition(0);
                    intake.setPower(0);
                    shooter.stop();
                    setPathState(pedroPathscalRedV1.PathState.Out);
                }
                break;
            case Out:
                follower.followPath(outOfZone);
                break;
        }
    }
    public void setPathState(pedroPathscalRedV1.PathState newState){
        pathState = newState;
        pathTimer.resetTimer();

        shotsTriggered = false;
    }
    @Override
    public void init(){
        pathState = pedroPathscalRedV1.PathState.Drive_Start2Shoot;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        shooter.init(hardwareMap);
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