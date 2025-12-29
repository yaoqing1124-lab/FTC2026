package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.FTC2025.RobotBase;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(11.3)
            .forwardZeroPowerAcceleration(-45.07734296205634)
            .lateralZeroPowerAcceleration(-60.711717632527126)
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.3,0,0.05,0.06))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.1,0,0,0.001))
            .headingPIDFCoefficients(new PIDFCoefficients(1,0,0.1,0.08))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.2,0,0,0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.03,0,0.0003,0,0.07))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.01,0,0,0,0))
            .centripetalScaling(0);
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("rf")
            .rightRearMotorName("rb")
            .leftRearMotorName("lb")
            .leftFrontMotorName("lf")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(59.357163316621566)
            .yVelocity(44.4441437007874);
    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2.55)
            .strafePodX(2.16)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
//            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD);
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer((localizerConstants))
                .build();
    }
}