package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp
//@Disabled
public class BLUEMain100Percent extends LinearOpMode {
    //test Abby 3
    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotorEx leftFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx rightBack = null;
    private DcMotorEx intake = null;
    private DcMotorEx shooter = null;
    private DcMotorEx ramp = null;
    private DcMotorEx kickStand = null;
    private Servo blocker=null;
    private Servo rightLight=null;
    private Servo leftLight=null;
    private Servo leftArm=null;
    private Servo rightArm=null;
    private CRServo rightIntakeServo=null;


    boolean shooterActive=false;
    boolean dpadDownPressed = false;
    boolean dpadUpPressed = false;
    boolean rightBumperPressed = false;
    boolean aPressed = false;
    boolean bPressed = false;
    boolean kickStandUp = false;
    boolean rampMoving1 = false;
    boolean rampMoving2 = false;

    int rampTargetPosition = 0;
    double velocity = 1300;
    int drivingSpeed=5000;
    int adjustment=0;
    int kickUP = 700;
    int kickDown = 0;

    double servoPosition = 0.0;
    double distance;
    double angleToGoalDegrees;
    double angleToGoalRadians;
    double limelightLensHeightInches = 16.75;
    double goalHeightInches = 29.5;
    double POI_Behind = 0.2;
    double POI_Up = 0.25;
    double integralSum=0;
    double Kp=0.04; //0.05
    double Ki=0.0; //0.005
    double Kd=0.0; //0.005
    private double lastError=0;

    Pose3D botpose;
    LLResult llResult;
    SparkFunOTOS otos;
    Limelight3A limelight;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        ramp = hardwareMap.get(DcMotorEx.class, "ramp");
        kickStand = hardwareMap.get(DcMotorEx.class, "kickStand");
        blocker = hardwareMap.get(Servo.class, "blocker");
        rightLight = hardwareMap.get(Servo.class, "rightLight");
        leftLight = hardwareMap.get(Servo.class, "leftLight");
        leftArm = hardwareMap.get(Servo.class, "leftArm");
        rightArm = hardwareMap.get(Servo.class, "rightArm");
        rightIntakeServo = hardwareMap.get(CRServo.class, "rightIntakeServo");
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        //reset encoder
        leftFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        ramp.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        kickStand.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //brake motors
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        ramp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        kickStand.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



        shooter.setVelocityPIDFCoefficients(100, 2, 60, 0);

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
        kickStand.setDirection(DcMotor.Direction.FORWARD);
        shooter.setDirection(DcMotor.Direction.REVERSE);
        blocker.setDirection(Servo.Direction.REVERSE);
        rightArm.setDirection(Servo.Direction.REVERSE);
        rightIntakeServo.setDirection(CRServo.Direction.REVERSE);
        otos.calibrateImu();
        otos.resetTracking();
        SparkFunOTOS.Pose2D currentPosition = new SparkFunOTOS.Pose2D(0, 0, 0);
        otos.setPosition(currentPosition);
        limelight.pipelineSwitch(0);

        // Retrieve the IMU from the hardware map
        IMU imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);
        limelight.start();

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();
        leftFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        leftBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightFront.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        rightBack.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        ramp.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        kickStand.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        blocker.setPosition(0.29);
        rightLight.setPosition(1);
        leftLight.setPosition(1);
        leftArm.setPosition(0.0);
        rightArm.setPosition(0.0);

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
            llResult = limelight.getLatestResult();
            botpose = llResult.getBotpose_MT2();
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

            leftFront.setVelocity(frontLeftPower*drivingSpeed);
            leftBack.setVelocity(backLeftPower*drivingSpeed);
            rightFront.setVelocity(frontRightPower*drivingSpeed);
            rightBack.setVelocity(backRightPower*drivingSpeed);

            if(gamepad1.left_bumper){
                drivingSpeed=500;
            }
            else{
                drivingSpeed=2788;
            }

            intake();
            shooter();
            setSpeed();
            align();
            autoAlign();
            fullPark();

            telemetry.addData("otos heading:", Math.toRadians(otos.getPosition().h));
            telemetry.addData("Shooter:", velocity);
            double bp_x = botpose.getPosition().x;
            double bp_y = botpose.getPosition().y;
            double angle = Math.atan(x/y);
            telemetry.addData("Botpose X: ", bp_x);
            telemetry.addData("Botpose Y: ", bp_y);
            telemetry.addData("Angle: ", angle);
            telemetry.addData("Botpose", botpose.toString());
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
            rightIntakeServo.setPower(1);
        }
        else{
            // turn off intake
            if(!rampMoving1 && !rampMoving2 && !gamepad2.left_bumper){
                // turn off the ramp (if its not being told to run for other reason)
                ramp.setMotorDisable();
                intake.setMotorDisable();
                rightIntakeServo.setPower(0);
            }
        }
    }

    public void shooter(){
        if (gamepad2.left_bumper && !rightBumperPressed) {
            rightBumperPressed = true;
            intake.setMotorEnable();
            intake.setPower(0.5);
            ramp.setMotorEnable();
            ramp.setPower(.7);
            blocker.setPosition(0);
            rampMoving1 = true;
        } else if (!gamepad2.left_bumper && rightBumperPressed) {
            rightBumperPressed = false;
            blocker.setPosition(0.29);
            ramp.setPower(0);
            ramp.setMotorEnable();
            rampMoving1 = false;
        }
        else {
            if(rampMoving1 && ramp.getCurrentPosition() >= rampTargetPosition && !gamepad2.left_bumper){
                ramp.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                ramp.setMotorDisable();
                rampMoving1 = false;
            }
        }

        if(gamepad2.left_stick_y==-1){
            shooterActive=true;
        }
        if(gamepad2.left_stick_y==1){
            shooterActive=false;
        }

        if (gamepad2.dpad_up) {
            if (!dpadUpPressed) {
                // only do the 1st time button pressed
                adjustment+=10;
                dpadUpPressed=true;
            }
        } else {
            // button released
            dpadUpPressed=false;
        }

        if(gamepad2.dpad_down){
            if (!dpadDownPressed) {
                // only do the 1st time button pressed
                adjustment-=10;
                dpadDownPressed=true;
            }
        }
        else{
            dpadDownPressed=false;
        }

        if(shooterActive){
            shooter.setVelocity(velocity);
        }
        else{
            shooter.setVelocity(0);
        }
        if(gamepad2.a){
            leftArm.setPosition(.3);
        }
        else{
            leftArm.setPosition(0);
        }
        if(gamepad2.b){
            rightArm.setPosition(.3);
        }
        else{
            rightArm.setPosition(0);
        }
    }

    public void setSpeed(){
        if (llResult !=null && llResult.isValid()){
            angleToGoalDegrees = llResult.getTy();
            angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
        }
        if(Math.abs(shooter.getVelocity()-velocity)<40){
            rightLight.setPosition(0.5);
        }
        else{
            rightLight.setPosition(1);
        }
        if(distance>=1.5){
            velocity= 90*distance+1330;
        }
        else{
            velocity = 200*distance+1070;
        }
    }

    public void align(){
        LimelightTesting.TargetInfo target = getTargetInfo();
        if (target != null) {
            distance=target.distance;
            if(target.bearing<-5){
                leftLight.setPosition(.32);
            }
            else if(target.bearing>5){
                leftLight.setPosition(.611);
            }
            else if(Math.abs(target.bearing)<10 && Math.abs(target.bearing)>1){
                leftLight.setPosition(0.388);
            }
            else{
                leftLight.setPosition(0.5);
            }
            telemetry.addData("Bearing", "%.2f°", target.bearing);
            telemetry.addData("Distance", "%.3f m", target.distance);
            telemetry.update();
        } else {
            leftLight.setPosition(1);
            telemetry.addLine("No target");
        }
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
        if (gamepad1.a && !aPressed && llResult.isValid()) {
            LimelightTesting.TargetInfo info = getTargetInfo();
            if (info !=null) {
                err = Math.abs(info.bearing);
                while (err > 0.02 && opModeIsActive()) {
                    if (Math.abs(gamepad1.left_stick_y) > 0 || Math.abs(gamepad1.left_stick_x) > 0 || Math.abs(gamepad1.right_stick_x) > 0) {
                        break;
                    }
                    align();
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
        } else if (aPressed && !gamepad1.a) {
            // x was pressed, but not pressed any longer
            aPressed = false;
        }
    }

    public void fullPark(){
        if(gamepad1.b && !bPressed){
            kickStand.setMotorEnable();
            bPressed = true;
            if(!kickStandUp) {
                kickStand.setTargetPosition(kickUP);
                kickStandUp = true;
            }
            else {
                kickStand.setTargetPosition(kickDown);
                kickStandUp = false;
            }
            kickStand.setPower(.6);
            while(Math.abs(kickStand.getTargetPosition()-kickStand.getCurrentPosition())>10){
                kickStand.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
        }
        else if(!gamepad1.b && bPressed){
            bPressed = false;
            kickStand.setMotorDisable();
        }
    }

    public void turnRight(double rot){
        leftFront.setPower(rot);
        rightFront.setPower(-rot);
        leftBack.setPower(rot);
        rightBack.setPower(-rot);
    }

    public void turnLeft(double rot){
        leftFront.setPower(-rot);
        rightFront.setPower(rot);
        leftBack.setPower(-rot);
        rightBack.setPower(rot);
    }

}