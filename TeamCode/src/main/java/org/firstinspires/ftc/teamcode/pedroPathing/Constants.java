package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Constants {

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14.1)
            .forwardZeroPowerAcceleration(-45.33265128583838)
            .lateralZeroPowerAcceleration(-90.34636138484028)
            .centripetalScaling(0.0005)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.5, 0, .05, 0))
            .headingPIDFCoefficients(new PIDFCoefficients(2, 0, 0.005, 0))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.0045,0.001,0.0008,0.6,0));


    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .leftFrontMotorName("motorLF")
            .leftRearMotorName("motorLB")
            .rightFrontMotorName("motorRF")
            .rightRearMotorName("motorRB")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(338.9431)
            .yVelocity(253.1578);


    public static ThreeWheelConstants localizerConstants =
            new ThreeWheelConstants()
                    .forwardTicksToInches(.0030099120761076918)
                    .strafeTicksToInches(.0031341799797724655)
                    .turnTicksToInches(.0013543171886686441)
                    .leftPodY(4)
                    .rightPodY(-4)
                    .strafePodX(-4.25)
                    .leftEncoder_HardwareMapName("motorLF")
                    .rightEncoder_HardwareMapName("motorRB")
                    .strafeEncoder_HardwareMapName("motorLB")
                    .leftEncoderDirection(Encoder.FORWARD)
                    .rightEncoderDirection(Encoder.REVERSE)
                    .strafeEncoderDirection(Encoder.FORWARD);

    public static PathConstraints pathConstraints = new PathConstraints(
            0.995,
            0.1,
            0.1,
            0.007,
            500,
            5,
            10,
            1
    );

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .mecanumDrivetrain(driveConstants)
                .threeWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .build();
    }
}