package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    /*                  15 + 0
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.250)
            .forwardZeroPowerAcceleration(-81.42478097877667)
            .lateralZeroPowerAcceleration(-75.96693069797573)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.045, 0, 0.001, 0.025))
            .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0, 0.09, 0.015))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.005, 0, 0.0005, 1, 0.05))
            .useSecondaryHeadingPIDF(true)
            .useSecondaryTranslationalPIDF(true)
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.9, 0, 0.09, 0.005))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.05, 0, 0.01, 0.015))
            .centripetalScaling(0.00065);
     */

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(12.250)
            .forwardZeroPowerAcceleration(-81.42478097877667)
            .lateralZeroPowerAcceleration(-75.96693069797573)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.045, 0, 0.001, 0.025))
            .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0, 0.09, 0.015))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(1, 0.01, 0.00001, 0.2, 0.03))
            .useSecondaryHeadingPIDF(true)
            .useSecondaryTranslationalPIDF(true)
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.9, 0, 0.09, 0.005))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.05, 0, 0.01, 0.015))
            .centripetalScaling(0.00065);

   // public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.77, 1);
   public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 0.77, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("front_right")
            .rightRearMotorName("back_right")
            .leftRearMotorName("back_left")
            .leftFrontMotorName("front_left")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .xVelocity(91.82059316560039)
            .yVelocity(66.9713653804749)
            .useBrakeModeInTeleOp(true);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-2.07153543)
            .strafePodX(2.0388189)
            .distanceUnit(DistanceUnit.MM)
            .hardwareMapName("pinpoint")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}