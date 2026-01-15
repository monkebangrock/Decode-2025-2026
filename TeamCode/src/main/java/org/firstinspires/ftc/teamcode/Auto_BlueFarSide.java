package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
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
public class Auto_BlueFarSide extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private DcMotorEx rightShooter;
    private DcMotorEx ramp;
    private DcMotorEx intake;
    private int pathState;
    private Servo blocker;
    int velocity = 1000;

    private final Pose startPose = new Pose(21, 124, Math.toRadians(144));
    private final Pose launchPose1 = new Pose(42.5, 100, Math.toRadians(132));
    private final Pose launchPose2 = new Pose(45, 95, Math.toRadians(135));
    private final Pose launchPose3 = new Pose(50,90,Math.toRadians(131));
    private final Pose pickup1 = new Pose(51, 89, Math.toRadians(180));
    private final Pose pickup2 = new Pose(51, 64, Math.toRadians(180));
    private final Pose finishPickup1 = new Pose(24, 89, Math.toRadians(180));
    private final Pose finishPickup2 = new Pose(24, 64, Math.toRadians(180));
    private final Pose control = new Pose(47,60);
    private final Pose ending = new Pose(60,138,0);

    private Path scorePreload;
    private PathChain beforePickup1, getPickup1, scorePickup1, beforePickup2, getPickup2, scorePickup2, endPath;

    public void buildPaths() {
        /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
        scorePreload = new Path(new BezierLine(startPose, launchPose1));
        scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), launchPose1.getHeading());
    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */
        beforePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose2, pickup1))
                .setLinearHeadingInterpolation(launchPose2.getHeading(), pickup1.getHeading())
                .build();
        getPickup1 = follower.pathBuilder()
                .addPath(new BezierLine(pickup1, finishPickup1))
                .setLinearHeadingInterpolation(pickup1.getHeading(), finishPickup1.getHeading())
                .build();
        scorePickup1 = follower.pathBuilder()
                .addPath(new BezierLine(finishPickup1, launchPose2))
                .setLinearHeadingInterpolation(finishPickup1.getHeading(), launchPose2.getHeading())
                .build();
        beforePickup2 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose2, pickup2))
                .setLinearHeadingInterpolation(launchPose2.getHeading(), pickup2.getHeading())
                .build();
        getPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2, finishPickup2))
                .setLinearHeadingInterpolation(pickup2.getHeading(), finishPickup2.getHeading())
                .build();
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(finishPickup2, control, launchPose3))
                .setLinearHeadingInterpolation(finishPickup2.getHeading(), launchPose3.getHeading())
                .build();
        endPath = follower.pathBuilder()
                .addPath(new BezierLine(launchPose3, ending))
                .setLinearHeadingInterpolation(launchPose3.getHeading(), ending.getHeading())
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
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()){
                    stopShooter();
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
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        blocker = hardwareMap.get(Servo.class, "blocker");

        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightShooter.setDirection(DcMotor.Direction.REVERSE);
        ramp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ramp.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        rightShooter.setVelocityPIDFCoefficients(100, 2, 60, 0);

        waitForStart();
        opmodeTimer.resetTimer();
        blocker.setPosition(0.06);
        rightShooter.setVelocity(1250);
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
        blocker.setPosition(0.08);
        while((rightShooter.getVelocity() <= 1250)&&opModeIsActive()){
            telemetry.addData("velocity",rightShooter.getVelocity());
            rightShooter.setVelocity(1250);
            telemetry.update();
        }
        blocker.setPosition(0);
        ramp.setPower(1);
        double current = getRuntime();
        while((getRuntime()<current+3)&&opModeIsActive()){
            telemetry.addData("time:",(getRuntime()-current));
            telemetry.update();
        }
        //rightShooter.setVelocity(0);
        blocker.setPosition(0.06);
        //rightShooter.setPower(0);
        telemetry.addData("velocity",rightShooter.getVelocity());
        // pusher.setPower(0);
        ramp.setPower(0);
        //rightShooter.setMotorDisable();
    }

    public void startIntake(){
        intake.setPower(0.7);
        ramp.setPower(0.6);
    }

    public void endIntake(){
        intake.setPower(0);
        ramp.setPower(0);
    }

    public void stopShooter(){
        rightShooter.setVelocity(0);
        rightShooter.setPower(0);
        rightShooter.setMotorDisable();
    }
}

