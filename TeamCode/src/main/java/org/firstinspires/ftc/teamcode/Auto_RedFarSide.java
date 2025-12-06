package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class Auto_RedFarSide extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private DcMotorEx rightShooter;
    private DcMotorEx ramp;
    private DcMotorEx intake;
    private Servo tapper;
    private int pathState;
    int velocity = 1000;

    private final Pose startPose = new Pose(123, 124, Math.toRadians(216));
    private final Pose launchPose = new Pose(101.5, 105, Math.toRadians(236));
    private final Pose pickup1 = new Pose(96, 84, Math.toRadians(180));
    private final Pose pickup2 = new Pose(96, 54, Math.toRadians(180));
    private final Pose finishPickup1 = new Pose(125, 84, Math.toRadians(180));
    private final Pose finishPickup2 = new Pose(125, 59, Math.toRadians(180));
    private final Pose ending = new Pose(84,130,0);

    private Path scorePreload;
    private PathChain beforePickup1, getPickup1, scorePickup1, beforePickup2, getPickup2, scorePickup2, endPath;

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
        endPath = follower.pathBuilder()
                .addPath(new BezierLine(launchPose, ending))
                .setLinearHeadingInterpolation(launchPose.getHeading(), ending.getHeading())
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
                    shoot();
                    follower.followPath(beforePickup1);
                    setPathState(2);
                }
                break;
            case 2:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.followPath(getPickup1);
                    setPathState(3);
                }
                break;
            case 3:
                if (!follower.isBusy()) {
                    endIntake();
                    follower.followPath(scorePickup1);
                    setPathState(4);
                }
                break;
            case 4:
                if (!follower.isBusy()) {
                    shoot();
                    follower.followPath(beforePickup2);
                    setPathState(5);
                }
                break;
            case 5:
                if (!follower.isBusy()) {
                    startIntake();
                    follower.followPath(getPickup2);
                    setPathState(6);
                }
                break;
            case 6:
                if (!follower.isBusy()) {
                    endIntake();
                    follower.followPath(scorePickup2);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()){
                    shoot();
                    follower.followPath(endPath);
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
    public void runOpMode() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        ramp = hardwareMap.get(DcMotorEx.class, "ramp");
        tapper = hardwareMap.get(Servo.class, "tapper");
        intake = hardwareMap.get(DcMotorEx.class, "intake");

        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ramp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ramp.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        rightShooter.setVelocityPIDFCoefficients(53, 0.2, 0.5, 0);

        waitForStart();
        opmodeTimer.resetTimer();
        setPathState(0);
        while (opModeIsActive()) {
            follower.update();
            autonomousPathUpdate();

            // Feedback to Driver Hub
            telemetry.addData("path state", pathState);
            telemetry.addData("x", follower.getPose().getX());
            telemetry.addData("y", follower.getPose().getY());
            telemetry.addData("heading", follower.getPose().getHeading());
            telemetry.update();
        }
    }

    public void shoot() {
        rightShooter.setMotorEnable();
        while((rightShooter.getVelocity() != 1200)&&opModeIsActive()){
            telemetry.addData("velocity",rightShooter.getVelocity());
            rightShooter.setVelocity(1200);
            if(rightShooter.getVelocity()>=900){
                break;
            }
            telemetry.update();
        }
        //pusher.setPower(1);
        ramp.setPower(0.5);
        double current = getRuntime();
        while((getRuntime()<current+2)&&opModeIsActive()){
            if(getRuntime()>=current+0.8){
                tapper.setPosition(0.03);
            }
            telemetry.addData("time:",(getRuntime()-current));
            telemetry.update();
        }
        tapper.setPosition(0);
        while((rightShooter.getVelocity() != 0)&&opModeIsActive()){
            telemetry.addData("velocity",rightShooter.getVelocity());
            rightShooter.setVelocity(0);
            telemetry.update();
        }
        rightShooter.setPower(0);
        telemetry.addData("velocity",rightShooter.getVelocity());
        // pusher.setPower(0);
        ramp.setPower(0);
        rightShooter.setMotorDisable();
    }

    public void startIntake(){
        intake.setPower(0.5);
        ramp.setPower(0.75);
    }

    public void endIntake(){
        intake.setPower(0);
        ramp.setPower(0);
    }
}

