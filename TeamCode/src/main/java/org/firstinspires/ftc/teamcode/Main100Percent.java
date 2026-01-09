package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp
//@Disabled
public class Main100Percent extends LinearOpMode {
    //test Abby 3
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightBack = null;
    private DcMotorEx intake = null;
    private DcMotorEx rightShooter = null;
    private DcMotorEx leftShooter = null;
    private DcMotorEx ramp = null;
    private Servo blocker=null;
    boolean shooterActive=false;
    boolean dpadDownPressed = false;
    boolean dpadUpPressed = false;
    boolean rightBumperPressed = false;
    boolean xPressed = false;
    boolean rampMoving1 = false;
    boolean rampMoving2 = false;
    int rampTargetPosition = 0;
    int velocity = 1000;
    long timer = 0;
    double servoPosition = 0.0;
    SparkFunOTOS otos;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        rightShooter = hardwareMap.get(DcMotorEx.class, "rightShooter");
        leftShooter = hardwareMap.get(DcMotorEx.class, "leftShooter");
        ramp = hardwareMap.get(DcMotorEx.class, "ramp");
        blocker = hardwareMap.get(Servo.class, "blocker");
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");


        //reset encoder
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightShooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftShooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        ramp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //brake motors
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftShooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ramp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightShooter.setVelocityPIDFCoefficients(60, 2, 60, 0);


        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        ramp.setDirection(DcMotorSimple.Direction.FORWARD);
        rightShooter.setDirection(DcMotor.Direction.REVERSE);
        otos.calibrateImu();
        otos.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setPosition(currentPosition);


        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);


        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        leftFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        ramp.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        blocker.setPosition(0.06);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            /*scaling equation:
             * P = sign(x) * k * |x|^n
             * P: scaled value
             * x: gamepad input
             * sign(x): pos/neg of x
             * k: max power constant
             * n: reduces sensitivity for smaller values of x - greater value of n makes smaller values less powerful
             * */
            double y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            double yscaled = y != 0 ? Math.signum(y) * Math.pow(Math.abs(y), 2) : 0;
            double x = gamepad1.left_stick_x;
            double xscaled = x != 0 ? Math.signum(x) * Math.pow(Math.abs(x), 2) : 0;
            double rx = gamepad1.right_stick_x;
            double rxscaled = rx != 0 ? Math.signum(rx) * Math.pow(Math.abs(rx), 2) : 0;

            // This button choice was made so that it is hard to hit on accident,
            // it can be freely changed based on preference.
            // The equivalent button is start on Xbox-style controllers.
            if (gamepad1.x) {
                otos.calibrateImu();
                otos.resetTracking();
                otos.setPosition(currentPosition);
            }

            double botHeading = Math.toRadians(otos.getPosition().h);
            //imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);

            // Rotate the movement direction counter to the bot's rotation
            double rotX = xscaled * Math.cos(-botHeading) - yscaled * Math.sin(-botHeading);
            double rotY = xscaled * Math.sin(-botHeading) + yscaled * Math.cos(-botHeading);

            //rotX = rotX * 1.1;  // Counteract imperfect strafing

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
            double frontLeftPower = (rotY + rotX + rxscaled) / denominator;
            double backLeftPower = (rotY - rotX + rxscaled) / denominator;
            double frontRightPower = (rotY - rotX - rxscaled) / denominator;
            double backRightPower = (rotY + rotX - rxscaled) / denominator;

            leftFront.setPower(frontLeftPower);
            leftBack.setPower(backLeftPower);
            rightFront.setPower(frontRightPower);
            rightBack.setPower(backRightPower);

            intake();
            shooter();
            unjam();

            telemetry.addData("otos heading:", Math.toRadians(otos.getPosition().h));
            telemetry.addData("Shooter:", velocity);
            telemetry.addData("imu output: ", imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS));
            telemetry.update();
        }
    }

    public void intake(){
        if(gamepad1.right_bumper){
            // run both ramp and intake
            intake.setMotorEnable();
            ramp.setMotorEnable();
            intake.setPower(.5);
            ramp.setPower(.6);
        }
        else{
            // turn off intake
            intake.setMotorDisable();
            if(!rampMoving1 && !rampMoving2 && !gamepad2.right_bumper){
                // turn off the ramp (if its not being told to run for other reason)
                ramp.setMotorDisable();
            }
        }
    }

    public void shooter(){
        if (gamepad2.right_bumper && !rightBumperPressed) {
            rightBumperPressed = true;

            ramp.setMotorEnable();
            ramp.setPower(1);
            blocker.setPosition(0);
            rampMoving1 = true;
        } else if (!gamepad2.right_bumper && rightBumperPressed) {
            rightBumperPressed = false;
            blocker.setPosition(0.06);
            ramp.setPower(0);
            ramp.setMotorEnable();
            rampMoving1 = false;
        }
        else {
            if(rampMoving1 && ramp.getCurrentPosition() >= rampTargetPosition && !gamepad2.right_bumper){
                ramp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ramp.setMotorDisable();
                rampMoving1 = false;
            }
        }

        if(gamepad2.dpad_up && !dpadUpPressed){
            shooterActive=true;
            dpadUpPressed=true;
        }
        else{
            dpadUpPressed=false;
        }
        if(gamepad2.dpad_down && !dpadDownPressed){
            shooterActive=false;
            dpadUpPressed=true;
        }
        else{
            dpadUpPressed=false;
        }
        if(shooterActive){
            rightShooter.setVelocity(velocity);
        }
        else{
            rightShooter.setVelocity(0);
        }
        if(gamepad2.left_stick_y<0){
            velocity-=gamepad2.left_stick_y*10;
        }
        else if(gamepad2.left_stick_y>0){
            velocity-=gamepad2.left_stick_y*10;
        }
    }

    public void unjam(){
        if(gamepad2.x && !xPressed) {
            // on first pressing x
            xPressed = true;
            rampMoving2 = true;
            rampTargetPosition = ramp.getCurrentPosition() - 100;
            ramp.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            ramp.setPower(1.0);
            ramp.setTargetPosition(rampTargetPosition);
            ramp.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            ramp.setMotorEnable();
        } else if (xPressed && !gamepad2.x) {
            // x was pressed, but not pressed any longer
            xPressed = false;
        } else {
            if (rampMoving2 && ramp.getCurrentPosition() <= rampTargetPosition &&!gamepad2.right_bumper){
                // Done moving ramp -- go back to RUN_WITHOUT_ENCODER mode
                ramp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ramp.setMotorDisable();
                rampMoving2 = false;
            }
        }
    }

    public void turnRight(int deg){
        int target = deg*600;
        while (leftFront.getCurrentPosition() < leftFront.getTargetPosition() && opModeIsActive()) {
            leftFront.setVelocity(1000);
            rightFront.setVelocity(1000);
            leftBack.setVelocity(1000);
            rightBack.setVelocity(1000);
            leftFront.setTargetPosition(target);
            rightFront.setTargetPosition(target);
            leftBack.setTargetPosition(target);
            rightBack.setTargetPosition(target);
            telemetry.update();
        }
    }

    public void turnLeft(int deg){
        int target = deg*600;
        while (leftFront.getCurrentPosition() < leftFront.getTargetPosition() && opModeIsActive()) {
            leftFront.setVelocity(-1000);
            rightFront.setVelocity(-1000);
            leftBack.setVelocity(-1000);
            rightBack.setVelocity(-1000);
            leftFront.setTargetPosition(target);
            rightFront.setTargetPosition(target);
            leftBack.setTargetPosition(target);
            rightBack.setTargetPosition(target);
            telemetry.update();
        }
    }

}