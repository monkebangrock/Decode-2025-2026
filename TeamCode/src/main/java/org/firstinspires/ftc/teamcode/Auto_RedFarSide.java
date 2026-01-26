package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.List;

@Autonomous
public class Auto_RedFarSide extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer;
    private DcMotorEx shooter;
    private DcMotorEx ramp;
    private DcMotorEx intake;
    private DcMotorEx leftFront;
    private DcMotorEx leftBack;
    private DcMotorEx rightFront;
    private DcMotorEx rightBack;
    private int pathState;
    private Servo blocker;
    SparkFunOTOS otos;
    Limelight3A limelight;
    double POI_Behind = 0.2;
    double POI_Up = 0.25;
    double integralSum=0;
    double Kp=0.04; //0.05
    double Ki=0.0; //0.005
    double Kd=0.0; //0.005
    private double lastError=0;

    private final Pose startPose = new Pose(123, 124, Math.toRadians(36));
    private final Pose launchPose1 = new Pose(93, 90, Math.toRadians(50));
    private final Pose launchPose2 = new Pose(91, 86, Math.toRadians(42));
    private final Pose launchPose3 = new Pose(90,87,Math.toRadians(42));
    private final Pose launchPose4 = new Pose(79,95,Math.toRadians(29));
    private final Pose pickup1 = new Pose(93, 88, Math.toRadians(0));
    private final Pose pickup2 = new Pose(94, 67, Math.toRadians(1));
    private final Pose pickup3 = new Pose(94, 40, Math.toRadians(1));
    private final Pose finishPickup1 = new Pose(128, 88, Math.toRadians(0));
    private final Pose finishPickup2 = new Pose(132, 67, Math.toRadians(0));
    private final Pose finishPickup3 = new Pose(131, 40, Math.toRadians(0));
    private final Pose control = new Pose(80,69);
    private final Pose control1 = new Pose(97,60);
    private final Pose control2 = new Pose(80,45);
    private final Pose ending = new Pose(79,100,Math.toRadians(29));

    private Path scorePreload;
    private PathChain beforePickup1, getPickup1, scorePickup1, beforePickup2, getPickup2, scorePickup2, beforePickup3, getPickup3, scorePickup3,endPath;

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
                .addPath(new BezierCurve(launchPose2,control,pickup2))
                .setLinearHeadingInterpolation(launchPose2.getHeading(), pickup2.getHeading())
                .build();
        getPickup2 = follower.pathBuilder()
                .addPath(new BezierLine(pickup2, finishPickup2))
                .setLinearHeadingInterpolation(pickup2.getHeading(), finishPickup2.getHeading())
                .build();
        scorePickup2 = follower.pathBuilder()
                .addPath(new BezierCurve(finishPickup2, control1, launchPose3))
                .setLinearHeadingInterpolation(finishPickup2.getHeading(), launchPose3.getHeading())
                .build();
        beforePickup3 = follower.pathBuilder()
                .addPath(new BezierCurve(launchPose3, control2, pickup3))
                .setLinearHeadingInterpolation(launchPose3.getHeading(), pickup3.getHeading())
                .build();
        getPickup3 = follower.pathBuilder()
                .addPath(new BezierLine(pickup3, finishPickup3))
                .setLinearHeadingInterpolation(pickup3.getHeading(), finishPickup3.getHeading())
                .build();
        scorePickup3 = follower.pathBuilder()
                .addPath(new BezierLine(finishPickup3, launchPose4))
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
                shooter.setVelocity(1270);
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
                    startIntake();
                    follower.followPath(getPickup3);
                    setPathState(9);
                }
                break;
            case 9:
                if (!follower.isBusy()) {
                    //endIntake();
                    follower.followPath(scorePickup3);
                    setPathState(10);
                }
                break;
            case 10:
                if (!follower.isBusy()){
                    shoot();
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
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        ramp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ramp.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        blocker.setDirection(Servo.Direction.REVERSE);

        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);

        shooter.setVelocityPIDFCoefficients(100, 2, 60, 0);
        limelight.pipelineSwitch(1);
        limelight.start();

        waitForStart();
        opmodeTimer.resetTimer();
        blocker.setPosition(0.29);
        shooter.setVelocity(1260);
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
        blocker.setPosition(0.29);
        while((shooter.getVelocity() <= 1260)&&opModeIsActive()){
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
        blocker.setPosition(0.29);
        //shooter.setPower(0);
        telemetry.addData("velocity",shooter.getVelocity());
        // pusher.setPower(0);
        ramp.setPower(0);
        endIntake();
        //shooter.setMotorDisable();
    }

    public void startIntake(){
        intake.setPower(0.45);
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

    public LimelightTesting.TargetInfo getTargetInfo() {
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            return null;
        }
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials.isEmpty()) {
            return null;
        }
        LLResultTypes.FiducialResult tag = fiducials.get(0);
        Pose3D robotInTarget = tag.getRobotPoseTargetSpace();
        if (robotInTarget == null) {
            return null;
        }

        // Robot position in tag space
        double robotX = robotInTarget.getPosition().x;
        double robotY = robotInTarget.getPosition().y;
        double robotZ = robotInTarget.getPosition().z;

        // Robot heading (pitch is turn axis in this coordinate system)
        double robotHeading = robotInTarget.getOrientation().getPitch();

        // Vector from robot to POI (in tag space)
        double toPoiX = 0 - robotX;
        double toPoiY = POI_Up - robotY;
        double toPoiZ = POI_Behind - robotZ;

        // Direction to POI, minus robot heading = relative bearing
        double bearing = Math.toDegrees(Math.atan2(toPoiX, toPoiZ)) - robotHeading;

        // Normalize to -180 to 180
        while (bearing > 180) bearing -= 360;
        while (bearing < -180) bearing += 360;

        // 3D distance
        double distance = Math.sqrt(toPoiX * toPoiX + toPoiY * toPoiY + toPoiZ * toPoiZ);

        return new LimelightTesting.TargetInfo(bearing, distance);
    }

    public double PIDControl(double ref, double state){
        double error = ref-state;
        integralSum+= error * runtime.seconds();
        double derivative = (error-lastError)/runtime.seconds();
        lastError=error;
        runtime.reset();
        double output = (error*Kp)+(derivative*Kd)+(integralSum*Ki);
        return output;
    }

    public void autoAlign(){
        double err=0.0;
        LimelightTesting.TargetInfo info = getTargetInfo();
        if (info !=null) {
            err = Math.abs(info.bearing);
            while (err > 0.02 && opModeIsActive()) {
                if (Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.left_stick_x) > 0 || Math.abs(gamepad1.right_stick_x) > 0) {
                    break;
                }
                double power = PIDControl(0.0, info.bearing);
                // max power
                if (power > 0.5)
                    power =0.5;
                if (power < -0.5)
                    power = -0.5;
                // min power
                if (power >0 && power < 0.05 )
                    power = 0.05;
                if (power <0 && power > -0.05 )
                    power = -0.05;

                leftFront.setPower(-power);
                rightFront.setPower(power);
                leftBack.setPower(-power);
                rightBack.setPower(power);
                do {
                    info = getTargetInfo();
                    if (Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.left_stick_x) > 0 || Math.abs(gamepad1.right_stick_x) > 0) {
                        break;
                    }
                } while (info==null);
                err = Math.abs(info.bearing);
            }
        }
    }
}