package org.firstinspires.ftc.teamcode.opModes.tests;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.Subsystems.testTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

public class turretMotion extends NextFTCOpMode {

    public turretMotion() {
        addComponents(
                new SubsystemComponent(testTurret.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    private Pose predictPose;

    @Override
    public void onInit() {

    }

    @Override
    public void onUpdate() {
        double x = PedroComponent.follower().getPose().getX();
        double y = PedroComponent.follower().getPose().getY();
        double heading = PedroComponent.follower().getPose().getHeading();
        double vx = PedroComponent.follower().getVelocity().getXComponent();
        double vy = PedroComponent.follower().getVelocity().getYComponent();

        testTurret.INSTANCE.alignTurretWithMotion(x, y, heading, vx, vy, testTurret.INSTANCE.currentTicks, false);

    }
}
