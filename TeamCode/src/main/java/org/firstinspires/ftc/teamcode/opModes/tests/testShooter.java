package org.firstinspires.ftc.teamcode.opModes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mecanismos.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Led;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name="testShooter", group = "OpModes Tests")
@Configurable
public class testShooter extends NextFTCOpMode {
    public testShooter() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }
    private boolean shooterToggle = false;
    private boolean lastButtonState = false;
    public static double testHood = 0;

    MecanumDrive mecanumDrive = new MecanumDrive();

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
        mecanumDrive.start();
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

        if (gamepad1.left_trigger > 0.1) {
            Intake.INSTANCE.colet().invoke();
        } else {
            Intake.INSTANCE.stop();
        }
        if (shooterToggle) {
            Shooter.INSTANCE.fixedVelocity().invoke();
            Intake.INSTANCE.unlocked.invoke();
        } else {
            Shooter.INSTANCE.stopTeleOp();
            Intake.INSTANCE.locked.invoke();
        }

      //  Shooter.INSTANCE.testHood(testHood);
        telemetry.addData("Intake", "-----------7-------");
        telemetry.addData("Target Intke", Intake.INSTANCE.velocity);
        telemetry.addData("Current Intake", Intake.INSTANCE.currentVelocity);
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
