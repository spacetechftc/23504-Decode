package org.firstinspires.ftc.teamcode.opModes.tests;

import org.firstinspires.ftc.teamcode.Subsystems.testTurret;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

public class testingTurret extends NextFTCOpMode {

    public static int pos = 0;

    public testingTurret() {
        addComponents(
                new SubsystemComponent(testTurret.INSTANCE),
                BindingsComponent.INSTANCE,
                BulkReadComponent.INSTANCE
        );
    }

    @Override
    public void onInit() {

    }

    @Override
    public void onUpdate() {
        testTurret.INSTANCE.turretToPosition(pos);

        telemetry.addData("Posição", pos);
        telemetry.update();
    }
}
