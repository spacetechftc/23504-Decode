package org.firstinspires.ftc.teamcode.opModes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Led;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name="testShooter", group = "OpModes Tests")
@Configurable
public class testShooter extends NextFTCOpMode {
    public testShooter() {
        addComponents(
                new SubsystemComponent(Shooter.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE
        );
    }
    private boolean shooterToggle = false;
    private boolean lastButtonState = false;
    public static double testHood = 0;

    @Override
    public void onInit() {

    }

    @Override
    public void onWaitForStart() {
        telemetry.addData("READY", "LET'S GO SPACETECH!");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {

    }

    @Override
    public void onUpdate() {
        // Estado do Toggle do shooter
        boolean currentButtonState = gamepad1.right_bumper;

        // Detecta a transição de "solto" -> "pressionado"
        if (currentButtonState && !lastButtonState) {
            shooterToggle = !shooterToggle; // inverte o estado do toggle
        }
        lastButtonState = currentButtonState;

        if (shooterToggle) {
            Shooter.INSTANCE.fixedVelocity().invoke();
        } else {
            Shooter.INSTANCE.stopTeleOp();
        }

      //  Shooter.INSTANCE.testHood(testHood);

        telemetry.addData("Shooter", "-----------7-------");
        telemetry.addData("Target Shooter", Shooter.INSTANCE.velocity);
        telemetry.addData("Current Shooter", Shooter.INSTANCE.currentVelocity);
        telemetry.update();
        BindingManager.update();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }
}
