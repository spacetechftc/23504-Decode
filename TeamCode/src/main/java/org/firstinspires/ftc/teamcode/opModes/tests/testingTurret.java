package org.firstinspires.ftc.teamcode.opModes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mecanismos.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.testTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name="testingTurret", group = "OpModes Tests")
@Configurable
public class testingTurret extends NextFTCOpMode {

    public static int pos = 0;

    public testingTurret() {
        addComponents(
                new SubsystemComponent(testTurret.INSTANCE),
                BindingsComponent.INSTANCE,
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    MecanumDrive mecanumDrive = new MecanumDrive();
    @Override
    public void onInit() {
        testTurret.INSTANCE.resetEncoder();
    }

    @Override
    public void onStartButtonPressed() {
        mecanumDrive.start();
    }

    @Override
    public void onUpdate() {
       // testTurret.INSTANCE.turretToPosition(pos);
        testTurret.INSTANCE.alignTurretTeleOp(PedroComponent.follower().getPose().getX(), PedroComponent.follower().getPose().getY(), PedroComponent.follower().getPose().getHeading(),PedroComponent.follower().getVelocity().getXComponent(),PedroComponent.follower().getVelocity().getYComponent(), testTurret.INSTANCE.currentTicks, false);

        telemetry.addData("Posição", pos);
        telemetry.update();
    }
}
