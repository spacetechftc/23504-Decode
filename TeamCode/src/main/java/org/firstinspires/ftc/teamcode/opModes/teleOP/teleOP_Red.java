package org.firstinspires.ftc.teamcode.opModes.teleOP;

import static org.firstinspires.ftc.teamcode.opModes.Autonomous.Autonomous_Red_Close.autoEndPose;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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

@TeleOp(name="TeleOp - Red", group = "Official")
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

    private boolean turretMidToggle = false;
    private boolean turretLastButtonState = false;

    public Pose predictPose;

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
        double vx = PedroComponent.follower().getVelocity().getXComponent();
        double vy = PedroComponent.follower().getVelocity().getYComponent();


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
        if (turretMidToggle) {
            Turret.INSTANCE.turretToMid();
        } else {
            Turret.INSTANCE.alignTurret(x, y, heading, Turret.INSTANCE.currentTicks, false); // Torreta automática
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
