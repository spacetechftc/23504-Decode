package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
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
/*
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14.75)
            //.forwardZeroPowerAcceleration(-81.42478097877667)
          //  .lateralZeroPowerAcceleration(-75.96693069797573)
           // .translationalPIDFCoefficients(new PIDFCoefficients(0.045, 0, 0.001, 0.025))
          //  .drivePIDFCoefficients(new FilteredPIDFCoefficients(1, 0.01, 0.00001, 0.2, 0.03))
           // .drivePIDFCoefficients(new FilteredPIDFCoefficients(2, 0.1, 0.0000001, 0, 0.02))
            //.secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.05, 0, 0.01, 0.015))
          //  .useSecondaryTranslationalPIDF(true)
            .headingPIDFCoefficients(new PIDFCoefficients(0.9, 0, 0.09, 0.015))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.9, 0, 0.09, 0.005))
            .useSecondaryHeadingPIDF(true)
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.06, 0.21736838559376925, 0.00149214893232419))
            .centripetalScaling(0);

 */

    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(14.75)
            /*.forwardZeroPowerAcceleration(-27.153819082146313)
            .lateralZeroPowerAcceleration(-55.431987187032966)
            .holdPointHeadingScaling(0.35)
            .holdPointTranslationalScaling(0.35)

            .headingPIDFCoefficients(new PIDFCoefficients(1.4, 0, 0.08, 0))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(0.6, 0, 0.08, 0))
            .useSecondaryHeadingPIDF(true)

            .translationalPIDFCoefficients(new PIDFCoefficients(0.1, 0, 0.002, 0.0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.2, 0, 0.012, 0.0))
            .useSecondaryTranslationalPIDF(true)

            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.01, 0, 0.0006, 0.6, 0))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.02, 0.0, 0.0008, 0.6, 0.0))

             */

            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.08, 0.21736838559376925, 0.00149214893232419))
            .centripetalScaling(0);

    public static PathConstraints pathConstraints = new PathConstraints(0.95, 100, 0.9, 1);

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
            .xVelocity(73.73652324526329)
            .yVelocity(58.26164233590674)
            .useVoltageCompensation(true)
            .nominalVoltage(12.7)
            .useBrakeModeInTeleOp(true);

    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(2.471615288201278)
            .strafePodX(-1.4377056932824814)
            .distanceUnit(DistanceUnit.INCH)
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