package org.firstinspires.ftc.teamcode.opModes.tests;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mecanismos.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.WebcamSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "Test-Webcam", group = "Tests")
public class TestWebCam extends NextFTCOpMode {
    public TestWebCam() {
        addComponents(
                new SubsystemComponent(WebcamSubsystem.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    MecanumDrive mecanumDrive = new MecanumDrive();

    private final Pose startPose = new Pose(0,0,0);

    @Override
    public void onInit() {
        PedroComponent.follower().setStartingPose(startPose);
        PedroComponent.follower().update();
    }

    @Override
    public void onStartButtonPressed() {
        mecanumDrive.start();
    }

    @Override
    public void onUpdate() {
        // Instância da odometria
        double x = PedroComponent.follower().getPose().getX();
        double y = PedroComponent.follower().getPose().getY();
        double heading = PedroComponent.follower().getPose().getHeading();

        // Telemetry
        telemetry.addData("Odometria", "------------------");
        telemetry.addData("X", x);
        telemetry.addData("Y", y);
        telemetry.addData("Heading", Math.toDegrees(heading));

        telemetry.addData("Câmera", "------------------");
        telemetry.addData("X", WebcamSubsystem.INSTANCE.x);
        telemetry.addData("Y", WebcamSubsystem.INSTANCE.y);
        telemetry.update();
    }
}
