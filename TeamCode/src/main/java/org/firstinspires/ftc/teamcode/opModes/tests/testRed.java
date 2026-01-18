package org.firstinspires.ftc.teamcode.opModes.tests;

import static org.firstinspires.ftc.teamcode.opModes.Autonomous.Autonomous_Red_Close.autoEndPose;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mecanismos.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.testTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name="testRed", group = "Official")
public class testRed extends NextFTCOpMode {
    public testRed() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, LimelightSubsystem.INSTANCE, testTurret.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }
    MecanumDrive mecanumDrive = new MecanumDrive();
    private boolean shooterToggle = false;
    private boolean lastButtonState = false;

    private boolean turretMidToggle = false;
    private boolean turretLastButtonState = false;

    private boolean turretManualToggle = false;
    private boolean turretLastButtonStateManual = false;

    @Override
    public void onInit() {
        PedroComponent.follower().setStartingPose(autoEndPose == null ? new Pose() : autoEndPose);
        PedroComponent.follower().updatePose();
        new InstantCommand(() -> {
            LimelightSubsystem.INSTANCE.switchIndexRed.invoke();
        });
    }

    @Override
    public void onWaitForStart() {
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
        // Instância da odometria
        double x = PedroComponent.follower().getPose().getX();
        double y = PedroComponent.follower().getPose().getY();
        double heading = PedroComponent.follower().getPose().getHeading();
        // Estado do Toggle do shooter
        boolean currentButtonState = gamepad1.right_bumper;

        // Detecta a transição de "solto" -> "pressionado"
        if (currentButtonState && !lastButtonState) {
            shooterToggle = !shooterToggle; // inverte o estado do toggle
        }
        lastButtonState = currentButtonState;
//----------------------------------------------------------------------------------------------------
        // Estado da Torreta no meio
        boolean currentButtonStateTurret = gamepad1.x;

        // Detecta a transição de "solto" -> "pressionado"
        if (currentButtonStateTurret && !turretLastButtonState) {
            turretMidToggle = !turretMidToggle; // inverte o estado do toggle
        }
        turretLastButtonState = currentButtonStateTurret;

//----------------------------------------------------------------------------------------------------
        // Estado da Torreta controle Manual
        boolean currentButtonStateTurretManual = gamepad1.right_stick_button;

        // Detecta a transição de "solto" -> "pressionado"
        if (currentButtonStateTurretManual && !turretLastButtonStateManual) {
            turretManualToggle = !turretManualToggle; // inverte o estado do toggle
        }
        turretLastButtonStateManual = currentButtonStateTurretManual;


        // Lógica Intake e Shooter
        if (gamepad1.left_trigger > 0.1 || shooterToggle || gamepad1.b) {
            if (gamepad1.left_trigger > 0.1) {
                Intake.INSTANCE.colet().invoke();
            } else {
                Intake.INSTANCE.stop();
            }
            if (shooterToggle) {
                if (turretMidToggle) {
                    Shooter.INSTANCE.fixedVelocity().invoke();
                } else {
                    Shooter.INSTANCE.shooterOn().invoke();
                }
            } else {
                Shooter.INSTANCE.shooterNegative().invoke();
            }
            // Botão de emergência, para ejetar todas as bolas
            if (gamepad1.b) {
                Intake.INSTANCE.eject().invoke();
                Shooter.INSTANCE.shooterNegative().invoke();
            }
        } else {
            Intake.INSTANCE.stop();
            Shooter.INSTANCE.stop();
        }

        // Se a torreta fica centralizada no meio ou fica se centralizando.
        if (turretManualToggle) {
            testTurret.INSTANCE.manualTurret(gamepad1.dpad_left, gamepad1.dpad_right);
        } else if (turretMidToggle) {
            testTurret.INSTANCE.turretToPosition(0);
        } else {
            testTurret.INSTANCE.alignTurret(x, y, heading, testTurret.INSTANCE.currentTicks, false); // Torreta automática
        }


        // Telemetry
        telemetry.addData("Odometria", "------------------");
        telemetry.addData("X", x);
        telemetry.addData("Y", y);
        telemetry.addData("Heading", Math.toDegrees(heading));

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
        telemetry.update();
        BindingManager.update();
    }

    @Override
    public void onStop() {
        autoEndPose = PedroComponent.follower().getPose();
        BindingManager.reset();
    }
}
