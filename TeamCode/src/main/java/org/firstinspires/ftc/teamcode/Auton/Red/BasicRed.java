package org.firstinspires.ftc.teamcode.Auton.Red;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Auton.Logics.FlywheelLogic;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@TeleOp
public class BasicRed extends OpMode {
    private Follower follower;
    private DcMotorEx intake;
    private FlywheelLogic shooter = new FlywheelLogic();
    private Timer pathTimer, opModeTimer;
    private boolean shotsTriggered = false;
    public RobotHardware hardware;

    private enum PathState {
        Start,
        ShootPreload,
        OutOfZone,
    }

    PathState pathState;
    public PathChain start;
    public PathChain outOfZone;

    public void buildPaths() {
        start = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(121.941, 126.627),

                                new Pose(96.238, 95.149)
                        )
                ).setLinearHeadingInterpolation(Math.toRadians(36), Math.toRadians(47))

                .build();

        outOfZone = follower.pathBuilder().addPath(
                        new BezierLine(
                                new Pose(96.238, 95.149),

                                new Pose(115.380, 95.744)
                        )
                ).setTangentHeadingInterpolation()
                .build();
    }

    public void statePathUpdate() {
        switch (pathState) {
            case Start:
                follower.followPath(start, true);
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
                    setPathState(PathState.OutOfZone);
                }
                break;
            case OutOfZone:
                follower.followPath(outOfZone);
                break;
        }
    }

    @Override
    public void init() {
        pathState = BasicRed.PathState.Start;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        shooter.init(hardwareMap);
    }

    private void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();

        shotsTriggered = false;
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
    }

    public void loop() {
        follower.update();
        shooter.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.addData("Path Time", pathTimer.getElapsedTimeSeconds());


    }
}
