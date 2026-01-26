package org.firstinspires.ftc.teamcode.Auton;

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
public class pedroPathscalV1 extends OpMode {


    public DcMotorEx intake;
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    //FLYWHEEL LOGIC SETUP
    private FlywheelLogic shooter = new FlywheelLogic();
    private boolean shotsTriggered = false;
    private RobotHardware hardware;

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
    private final Pose startPose = new Pose(59.29519450800915,7,Math.toRadians(90));
    private final Pose shootPose = new Pose(59.29519450800915,84.27459954233409,Math.toRadians(129));
    public PathChain intake1in;
    public PathChain intake1out;
    public PathChain intake2in;
    public PathChain clear;
    public PathChain intake2out;
    public PathChain intake3in;
    public PathChain intake3out;
    public PathChain outOfZone;
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

                                new Pose(14.663, 84.110)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        intake1out = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(14.663, 84.110),

                                new Pose(59.295, 84.110)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(129))
                .setReversed()
                .build();

        intake2in = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(59.295, 84.110),
                                new Pose(56.318, 66.212),
                                new Pose(41.098, 59.037),
                                new Pose(18.947, 59.643)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        clear = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(18.947, 59.643),

                                new Pose(16.320, 71.059)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        intake2out = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(16.320, 71.059),

                                new Pose(59.295, 84.110)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(129))
                .setReversed()
                .build();

        intake3in = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(59.295, 84.110),
                                new Pose(46.466, 30.951),
                                new Pose(12.968, 35.389)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        intake3out = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.968, 35.389),

                                new Pose(59.295, 84.110)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(129))

                .build();

        outOfZone = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(59.295, 84.110),

                                new Pose(44.565, 79.682)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(129), Math.toRadians(180))

                .build();

    }
    public void statePathUpdate(){
        switch (pathState) {
            case Drive_Start2Shoot:
                follower.followPath(driveStart, true);
                setPathState(PathState.ShootPreload);
                break;
            case ShootPreload:
                shooter.setTargetRPM(4200);

                if (shooter.atSpeed() && !shotsTriggered) {
                    intake.setPower(1);
                    hardware.servo.setPosition(0.67);
                    shotsTriggered = true;
                    pathTimer.resetTimer();
                }

                if (shotsTriggered && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    hardware.servo.setPosition(0);
                    intake.setPower(0);
                    shooter.stop();
                    setPathState(PathState.Intake1);
                }
                break;
            case Intake1:
                intake.setPower(1);
                follower.followPath(intake1in, true);
                intake.setPower(0);
                setPathState(PathState.InOutTake1);
                break;
            case InOutTake1:
                follower.followPath(intake1out, true);
                setPathState(PathState.Shoot2);
                break;
            case Shoot2:
                shooter.setTargetRPM(4200);

                if (shooter.atSpeed() && !shotsTriggered) {
                    intake.setPower(1);
                    hardware.servo.setPosition(0.67);
                    shotsTriggered = true;
                    pathTimer.resetTimer();
                }

                if (shotsTriggered && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    hardware.servo.setPosition(0);
                    intake.setPower(0);
                    shooter.stop();
                    setPathState(PathState.Intake2);
                }
                break;

            case Intake2:
                intake.setPower(1);
                follower.followPath(intake2in);
                intake.setPower(0);
                setPathState(PathState.Clear);
                break;
            case Clear:
                follower.followPath(clear);
                sleep(1000);
                setPathState(PathState.InOuttake2);
                break;
            case InOuttake2:
                follower.followPath(intake2out);
                setPathState(PathState.Shoot3);
                break;
            case Shoot3:
                shooter.setTargetRPM(4200);

                if (shooter.atSpeed() && !shotsTriggered) {
                    intake.setPower(1);
                    hardware.servo.setPosition(0.67);
                    shotsTriggered = true;
                    pathTimer.resetTimer();
                }

                if (shotsTriggered && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    hardware.servo.setPosition(0);
                    intake.setPower(0);
                    shooter.stop();
                    setPathState(PathState.Intake3);
                }
                break;
            case Intake3:
                intake.setPower(1);
                follower.followPath(intake3in);
                intake.setPower(0);
                setPathState(PathState.InOutTake3);
                break;
            case InOutTake3:
                follower.followPath(intake3out);
                setPathState(PathState.Shoot4);
                break;
            case Shoot4:
                shooter.setTargetRPM(4200);

                if (shooter.atSpeed() && !shotsTriggered) {
                    intake.setPower(1);
                    hardware.servo.setPosition(0.67);
                    shotsTriggered = true;
                    pathTimer.resetTimer();
                }

                if (shotsTriggered && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    hardware.servo.setPosition(0);
                    intake.setPower(0);
                    shooter.stop();
                    setPathState(PathState.Out);
                }
                break;
            case Out:
                follower.followPath(outOfZone);
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
