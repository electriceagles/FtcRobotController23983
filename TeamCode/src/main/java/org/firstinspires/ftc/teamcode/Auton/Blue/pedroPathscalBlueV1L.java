package org.firstinspires.ftc.teamcode.Auton.Blue;

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
import org.firstinspires.ftc.teamcode.Auton.Logics.TurretLogic;
import org.firstinspires.ftc.teamcode.Auton.Logics.LimeDistance;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@TeleOp
public class pedroPathscalBlueV1L extends OpMode {


    public DcMotorEx intake;
    private Follower follower;
    private Timer pathTimer, opModeTimer;
    //LOGIC SETUPS
    private final FlywheelLogic shooter = new FlywheelLogic();
    private final TurretLogic turretControl = new TurretLogic();
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
                                new Pose(59.295, 7.000),

                                new Pose(59.295, 84.275)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))

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
                ).setTangentHeadingInterpolation()
                .setReversed()
                .build();

        intake2in = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(59.295, 84.110),
                                new Pose(48.410, 60.798),
                                new Pose(32.860, 59.060),
                                new Pose(18.947, 59.643)
                        )
                ).setTangentHeadingInterpolation()

                .build();

        clear = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(18.947, 59.643),

                                new Pose(15.002, 72.378)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        intake2out = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(15.002, 72.378),

                                new Pose(59.295, 84.110)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .setReversed()
                .build();

        intake3in = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(59.295, 84.110),
                                new Pose(46.460, 30.957),
                                new Pose(12.968, 35.389)
                        )
                ).setConstantHeadingInterpolation(Math.toRadians(180))

                .build();

        intake3out = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(12.968, 35.389),

                                new Pose(59.295, 84.110)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

        outOfZone = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(59.295, 84.110),

                                new Pose(44.565, 79.682)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))

                .build();

    }
    public void statePathUpdate(){
        switch (pathState) {
            case Drive_Start2Shoot:
                follower.followPath(start, true);
                setPathState(PathState.ShootPreload);
                break;
            case ShootPreload:
                shooter.setTargetRPM(4200);

                if (shooter.atSpeed() ) {
                    intake.setPower(1);
                    hardware.servo.setPosition(0.67);
                    pathTimer.resetTimer();
                }

                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
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

                if (shooter.atSpeed()) {
                    intake.setPower(1);
                    hardware.servo.setPosition(0.67);
                    pathTimer.resetTimer();
                }

                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
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
                if (pathTimer.getElapsedTimeSeconds() >= 1) {
                    setPathState(PathState.InOuttake2);
                }
                break;
            case InOuttake2:
                follower.followPath(intake2out);
                setPathState(PathState.Shoot3);
                break;
            case Shoot3:
                shooter.setTargetRPM(4200);

                if (shooter.atSpeed()) {
                    intake.setPower(1);
                    hardware.servo.setPosition(0.67);
                    pathTimer.resetTimer();
                }

                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
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

                if (shooter.atSpeed()) {
                    intake.setPower(1);
                    hardware.servo.setPosition(0.67);
                    pathTimer.resetTimer();
                }

                if (pathTimer.getElapsedTimeSeconds() > 0.3) {
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
    }


    @Override
    public void init(){
        pathState = PathState.Drive_Start2Shoot;
        turretControl.init(hardwareMap);
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
    }
    public void start(){
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    @Override
    public void loop(){
        follower.update();
        turretControl.update();

        statePathUpdate();
        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading",follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());


    }
}
