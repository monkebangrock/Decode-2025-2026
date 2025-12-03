package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Example Auto", group = "Examples")
public class Auto_BlueFarSide extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private DcMotorEx rightShooter;
    private DcMotorEx ramp;
    private DcMotorEx intake;
    private Servo pusher;
    private int pathState;
    int velocity = 1000;

    private final Pose startPose = new Pose(21, 124, Math.toRadians(-36));
    private final Pose launchPose = new Pose(37.5, 105, Math.toRadians(-36));
    private final Pose pickup1 = new Pose(48, 84, Math.toRadians(180));
    private final Pose pickup2 = new Pose(48, 59, Math.toRadians(180));
    private final Pose finishPickup1 = new Pose(19, 84, Math.toRadians(180));
    private final Pose finishPickup2 = new Pose(19, 59, Math.toRadians(180));

    private Path scorePreload;
    private PathChain beforePickup1, getPickup1, scorePickup1, beforePickup2, getPickup2, scorePickup2;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, launchPose));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), launchPose.getHeading());
    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */
        beforePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, pickup1))
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickup1.getHeading())
                .build();
        getPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1, finishPickup1))
                .setLinearHeadingInterpolation(pickup1.getHeading(), finishPickup1.getHeading())
                .build();
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(finishPickup1, launchPose))
                .setLinearHeadingInterpolation(finishPickup1.getHeading(), launchPose.getHeading())
                .build();
        beforePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, pickup2))
                .setLinearHeadingInterpolation(launchPose.getHeading(), pickup2.getHeading())
                .build();
        getPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2, finishPickup2))
                .setLinearHeadingInterpolation(pickup2.getHeading(), finishPickup2.getHeading())
                .build();
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(finishPickup2, launchPose))
                .setLinearHeadingInterpolation(finishPickup2.getHeading(), launchPose.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                follower.followPath(scorePreload);
                setPathState(1);
                shoot();
                break;
            case 1:
                if (!follower.isBusy()) {
                    follower.followPath(beforePickup1);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    follower.followPath(getPickup1);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup1);
                    shoot();
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    follower.followPath(beforePickup2);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    follower.followPath(getPickup2);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    follower.followPath(scorePickup2);
                    shoot();
                    setPathState(-1);
                }
                break;
        }
    }

    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

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

    @Override
    public void init(){
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        ramp = hardwareMap.get(DcMotorEx.class, "ramp");
        pusher = hardwareMap.get(Servo.class, "pusher");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ramp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ramp.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        rightShooter.setVelocityPIDFCoefficients(53, 0.2, 0.5, 0);

        setPathState(0);
    }

    public void shoot() {
        while(rightShooter.getVelocity() != 1){
            rightShooter.setVelocity(1);
        }
        for(int i = 0; i<3; i++) {
            pusher.setPosition(1); //tbd
            ramp.setVelocity(0);
            //shoot
            pusher.setPosition(0); //tbd
            ramp.setVelocity(0.75);
            double current = getRuntime() + 3;
            while (getRuntime() < current) {
            }
        }
        rightShooter.setVelocity(0);
        ramp.setVelocity(0);
    }
}

