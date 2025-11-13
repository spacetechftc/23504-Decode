package org.firstinspires.ftc.teamcode.opModes.teleOP;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mecanismos.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name="TeleOp - Red")
public class teleOP_Red extends NextFTCOpMode {
    public teleOP_Red() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, LimelightSubsystem.INSTANCE, Turret.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }
    MecanumDrive mecanumDrive = new MecanumDrive();
    private boolean shooterToggle = false;
    private boolean lastButtonState = false;
    private TelemetryManager telemetryM;

    @Override
    public void onInit() {
        new InstantCommand(() -> {
            LimelightSubsystem.INSTANCE.switchIndexRed.invoke();
        });
    }

    @Override
    public void onWaitForStart() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        LimelightSubsystem.INSTANCE.switchIndexRed.invoke();
        telemetry.addData("READY", "LET'S GO SPACETECH!");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        mecanumDrive.start();

    }

    @Override
    public void onUpdate() {
        boolean currentButtonState = gamepad1.right_bumper; // escolhe o botão que quiser (A, B, X, Y...)

        // Detecta a transição de "solto" -> "pressionado"
        if (currentButtonState && !lastButtonState) {
            shooterToggle = !shooterToggle; // inverte o estado do toggle
        }
        lastButtonState = currentButtonState;

        // Lógica Intake e Shooter
        if (gamepad1.left_trigger > 0.1 || shooterToggle) {
            if (gamepad1.left_trigger > 0.1) {
                Intake.INSTANCE.colet().invoke();
            } else {
                Intake.INSTANCE.stop();
            }
            if (shooterToggle) {
                Shooter.INSTANCE.shooterOn().invoke();
            } else {
                Shooter.INSTANCE.shooterNegative().invoke();
            }
        } else {
            Intake.INSTANCE.stop();
            Shooter.INSTANCE.stop();
        }

        // Telemetry

        telemetry.addData("Limelight", "------------------");
        telemetry.addData("Distance", LimelightSubsystem.INSTANCE.distance);
        telemetry.addData("TX", LimelightSubsystem.INSTANCE.tx);
        telemetry.addData("TA", LimelightSubsystem.INSTANCE.ta);


        telemetry.addData("Intake", "------------------");
        telemetry.addData("Target Intake", Intake.velocity);
        telemetry.addData("Current Intake", Intake.INSTANCE.currentVelocity);

        telemetry.addData("Shooter", "------------------");
        telemetry.addData("Target Shooter", Shooter.INSTANCE.velocity);
        telemetry.addData("Current Shooter", Shooter.INSTANCE.currentVelocity);

        BindingManager.update();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }
}
