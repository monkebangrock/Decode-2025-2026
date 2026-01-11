package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;



@TeleOp
public class LimelightTesting extends OpMode {
    //calculate distance
    double distance;
    double angleToGoalDegrees;
    double angleToGoalRadians;
    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 16.75;
    // distance from the target to the floor
    double goalHeightInches = 29.5;



    private ElapsedTime runtime = new ElapsedTime();
    SparkFunOTOS otos;
    Limelight3A limelight;

    @Override
    public void init() {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        otos = hardwareMap.get(SparkFunOTOS.class, "otos");
        limelight.pipelineSwitch(0);
        otos.calibrateImu();
    }

    @Override
    public void start(){
        limelight.start();
    }

    @Override
    public void loop(){
        LLResult llResult = limelight.getLatestResult();
        if (llResult !=null && llResult.isValid()){
            Pose3D botpose = llResult.getBotpose_MT2();
            angleToGoalDegrees = llResult.getTy();
            angleToGoalRadians = angleToGoalDegrees * (Math.PI / 180.0);
            distance = (goalHeightInches-limelightLensHeightInches)/Math.tan(angleToGoalRadians);
            telemetry.addData("Calculated Distance", distance);
            telemetry.addData("Target X", llResult.getTx());
            telemetry.addData("Target area", llResult.getTa());
            telemetry.addData("Botpose", botpose.toString());
            telemetry.update();
        }

    }
}
