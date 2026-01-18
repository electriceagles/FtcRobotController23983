package org.firstinspires.ftc.teamcode.basicOpMode.Tea_andEthan_Auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.Hardware.RobotHardware;

import org.firstinspires.ftc.teamcode.basicOpMode.FlywheelLogic;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Disabled
@TeleOp
public class pedroPathscalV1.6.7 extends OpMode {


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
        ShootPreload2,
        Intake1,
        InOutTake1,
        Intake2,
        teleLineUp,


    }
    PathState pathState;
    private final Pose startPose = new Pose(59.29519450800915,7,Math.toRadians(90));
    private final Pose shootPose = new Pose(59.29519450800915,84.27459954233409,Math.toRadians(129));
    public PathChain intake1in;
    public PathChain intake1out;
    public PathChain intake2in;
    public PathChain teleLineUp;

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
        teleLineUp = follower.pathBuilder().addPath(
                        new BezierCurve(
                                new Pose(15.652, 59.643),
                                new Pose(18.605, 44.524),
                                new Pose(71.244, 62.307),
                                new Pose(75.624, 72.027),
                                new Pose(46.011, 97.098)
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
                shooter.setTargetRPM(4200);

                if (shooter.atSpeed() && !shotsTriggered) {
                    hardware.servo.setPosition(0.67);
                    shotsTriggered = true;
                    pathTimer.resetTimer();
                }

                if (shotsTriggered && pathTimer.getElapsedTimeSeconds() > 0.3) {
                    hardware.servo.setPosition(0);
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
                setPathState(PathState.ShootPreload2);
                break;
           case ShootPreload2:
               shooter.setTargetRPM(4200);

               if (shooter.atSpeed() && !shotsTriggered) {
                   hardware.servo.setPosition(0.67);
                   shotsTriggered = true;
                   pathTimer.resetTimer();
               }

               if (shotsTriggered && pathTimer.getElapsedTimeSeconds() > 0.3) {
                   hardware.servo.setPosition(0);
                   shooter.stop();
                   setPathState(PathState.Intake2);
               }
               break;

            case Intake2:
                intake.setPower(1);
                follower.followPath(intake2in);
                intake.setPower(0);
                setPathState(PathState.teleLineUp);
                break;
            case teleLineUp:
                follower.followPath(teleLineUp);
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
