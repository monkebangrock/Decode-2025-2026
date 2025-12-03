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

@Autonomous(name = "Example Auto", group = "Examples")
public class Auto_RedFarSide extends OpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;

    private final Pose startPose = new Pose(123, 124, Math.toRadians(216));
    private final Pose launchPose = new Pose(106.5, 105, Math.toRadians(236));
    private final Pose pickup1 = new Pose(96, 84, Math.toRadians(0));
    private final Pose pickup2 = new Pose(96, 59, Math.toRadians(0));
    private final Pose finishPickup1 = new Pose(125, 84, Math.toRadians(0));
    private final Pose finishPickup2 = new Pose(125, 59, Math.toRadians(0));

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
        setPathState(0);
    }



}
