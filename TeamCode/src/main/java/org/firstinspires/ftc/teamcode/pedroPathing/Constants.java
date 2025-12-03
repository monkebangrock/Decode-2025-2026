package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.OTOSConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(9.75)
            .forwardZeroPowerAcceleration(-24.3706)
            .lateralZeroPowerAcceleration(-51.9473)
            .centripetalScaling(0.0038)
            .useSecondaryDrivePIDF(true)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.15, 0, 0.01, 0.02))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.12, 0, 0.015, 0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(1.6, 0, 0.1, 0.02))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.5, 0, 0.1, 0.015))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0000000001, 0, 0.00001, 0, 0.3))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0.000001, 0.01, 0.6));

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(86.1053)
            .yVelocity(73.0398);

    // NOTE: Use the 22mm mount box
    //       Skinny bar towards back of robot.
    public static OTOSConstants localizerConstants = new OTOSConstants()
            .hardwareMapName("otos")
            .linearUnit(DistanceUnit.INCH)
            .angleUnit(AngleUnit.RADIANS)
            .linearScalar(1.17928816)
            .angularScalar(0.9799)
            .offset(new SparkFunOTOS.Pose2D(-(180.0/25.4), -(76.0/25.4), (Math.PI/2.0)));
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.5, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .OTOSLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }
}
