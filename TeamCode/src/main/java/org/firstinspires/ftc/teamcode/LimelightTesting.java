package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;

@TeleOp(name = "Limelight Test", group = "TeleOp")
public class LimelightTesting extends LinearOpMode {

    private Limelight3A limelight;

    // POI offset from AprilTag (in meters)
    private static final double POI_BEHIND = 0.2;
    private static final double POI_UP = 0.25;

    public static class TargetInfo {
        public final double bearing;   // degrees
        public final double distance;  // meters

        public TargetInfo(double bearing, double distance) {
            this.bearing = bearing;
            this.distance = distance;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();

        waitForStart();

        while (opModeIsActive()) {
            TargetInfo target = getTargetInfo();

            if (target != null) {
                telemetry.addData("Bearing", "%.2f°", target.bearing);
                telemetry.addData("Distance", "%.3f m", target.distance);
            } else {
                telemetry.addLine("No target");
            }

            telemetry.update();
        }

        limelight.stop();
    }

    public TargetInfo getTargetInfo() {
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
        double toPoiY = POI_UP - robotY;
        double toPoiZ = POI_BEHIND - robotZ;

        // Direction to POI, minus robot heading = relative bearing
        double bearing = Math.toDegrees(Math.atan2(toPoiX, toPoiZ)) - robotHeading;

        // Normalize to -180 to 180
        while (bearing > 180) bearing -= 360;
        while (bearing < -180) bearing += 360;

        // 3D distance
        double distance = Math.sqrt(toPoiX * toPoiX + toPoiY * toPoiY + toPoiZ * toPoiZ);

        return new TargetInfo(bearing, distance);
    }
}