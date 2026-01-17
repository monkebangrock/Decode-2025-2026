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
public class Auto_RedCloseSide extends LinearOpMode {

    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private DcMotorEx ramp;
    private DcMotorEx intake;
    private DcMotorEx shooter;
    private int pathState;
    private Servo blocker;
    int velocity = 1000;

    private final Pose startPose = new Pose(88, 8, Math.toRadians(90));
    private final Pose launchPose1 = new Pose(86, 10, Math.toRadians(71));
    private final Pose launchPose2 = new Pose(85.5, 26.5, Math.toRadians(62));
    private final Pose launchPose3 = new Pose(85, 29, Math.toRadians(60));
    private final Pose launchPose4 = new Pose(85, 115, Math.toRadians(29));
    private final Pose pickup1 = new Pose(103, 47, Math.toRadians(0));
    private final Pose pickup2 = new Pose(108,75 , Math.toRadians(0));
    private final Pose pickup3 = new Pose(108, 98, Math.toRadians(0));
    private final Pose finishPickup1 = new Pose(132, 47, Math.toRadians(0));
    private final Pose finishPickup2 = new Pose(132, 75, Math.toRadians(0));
    private final Pose finishPickup3 = new Pose(129, 98, Math.toRadians(0));
    private final Pose ending = new Pose(85,115,29);

    private Path scorePreload;
    private PathChain beforePickup1, getPickup1, scorePickup1, beforePickup2, getPickup2, scorePickup2, beforePickup3, getPickup3, scorePickup3, endPath;

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
                .addPath(new BezierCurve(finishPickup2, launchPose3))
                .setLinearHeadingInterpolation(finishPickup2.getHeading(), launchPose3.getHeading())
                .build();
        beforePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(launchPose3, pickup3))
                .setLinearHeadingInterpolation(launchPose3.getHeading(), pickup3.getHeading())
                .build();
        getPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3, finishPickup3))
                .setLinearHeadingInterpolation(pickup3.getHeading(), finishPickup3.getHeading())
                .build();
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(finishPickup3, launchPose4))
                .setLinearHeadingInterpolation(finishPickup3.getHeading(), launchPose4.getHeading())
                .build();
        endPath = follower.pathBuilder()
                .addPath(new BezierLine(launchPose4, ending))
                .setLinearHeadingInterpolation(launchPose4.getHeading(), ending.getHeading())
                .build();
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case 0:
                shooter.setVelocity(1470);
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
                    //endIntake();
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
                    //endIntake();
                    follower.followPath(scorePickup2);
                    setPathState(7);
                }
                break;
            case 7:
                if (!follower.isBusy()){
                    shoot();
                    follower.followPath(beforePickup3);
                    setPathState(8);
                }
                break;
            case 8:
                if (!follower.isBusy()) {
                    shooter.setVelocity(1270);
                    startIntake();
                    follower.followPath(getPickup3);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    //\
                    // '/.
                    // endIntake();
                    follower.followPath(scorePickup3);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()){
                    shoot1();
                    follower.followPath(endPath);
                    setPathState(11);
                }
                break;
            case 11:
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
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        ramp = hardwareMap.get(DcMotorEx.class, "ramp");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        blocker = hardwareMap.get(Servo.class, "blocker");

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        ramp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ramp.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        blocker.setDirection(Servo.Direction.REVERSE);

        shooter.setVelocityPIDFCoefficients(100, 2, 60, 0);

        waitForStart();
        opmodeTimer.resetTimer();
        blocker.setPosition(0.33);
        shooter.setVelocity(1470);
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
        shooter.setMotorEnable();
        blocker.setPosition(0.33);
        while((shooter.getVelocity() <= 1470)&&opModeIsActive()){
            telemetry.addData("velocity",shooter.getVelocity());
            shooter.setVelocity(1470);
            telemetry.update();
        }
        blocker.setPosition(0);
        ramp.setPower(1);
        startIntake();
        double current = getRuntime();
        while((getRuntime()<current+3)&&opModeIsActive()){
            telemetry.addData("time:",(getRuntime()-current));
            telemetry.update();
        }
        //shooter.setVelocity(0);
        blocker.setPosition(0.33);
        //shooter.setPower(0);
        telemetry.addData("velocity",shooter.getVelocity());
        // pusher.setPower(0);
        ramp.setPower(0);
        endIntake();
        //shooter.setMotorDisable();
    }

    public void shoot1() {
        shooter.setMotorEnable();
        blocker.setPosition(0.33);
        while((shooter.getVelocity() >= 1260)&&opModeIsActive()){
            telemetry.addData("velocity",shooter.getVelocity());
            shooter.setVelocity(1260);
            telemetry.update();
        }
        blocker.setPosition(0);
        ramp.setPower(1);
        startIntake();
        double current = getRuntime();
        while((getRuntime()<current+3)&&opModeIsActive()){
            telemetry.addData("time:",(getRuntime()-current));
            telemetry.update();
        }
        //shooter.setVelocity(0);
        blocker.setPosition(0.33);
        //shooter.setPower(0);
        telemetry.addData("velocity",shooter.getVelocity());
        // pusher.setPower(0);
        ramp.setPower(0);
        endIntake();
        //shooter.setMotorDisable();
    }

    public void startIntake(){
        intake.setPower(0.3);
        ramp.setPower(0.6);
    }

    public void endIntake(){
        intake.setPower(0);
        ramp.setPower(0);
    }

    public void stopShooter(){
        shooter.setVelocity(0);
        shooter.setPower(0);
        shooter.setMotorDisable();
    }
}