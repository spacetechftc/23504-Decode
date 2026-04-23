package org.firstinspires.ftc.teamcode.opModes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mecanismos.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Led;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.testTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name="testGraph", group = "OpModes Tests")
@Configurable
public class testGraph extends NextFTCOpMode {
    public testGraph() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, testTurret.INSTANCE, Shooter.INSTANCE, Led.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }
    private boolean shooterToggle = false;
    private boolean lastButtonState = false;
    public static double testHood = 0.51;

    MecanumDrive mecanumDrive = new MecanumDrive();

    private final Pose startPose = new Pose(127, -1.45, Math.toRadians(180));

    public static TelemetryManager telemetryMa;

    @Override
    public void onInit() {
        telemetryMa = PanelsTelemetry.INSTANCE.getTelemetry();
        testTurret.INSTANCE.resetEncoder();
        PedroComponent.follower().setStartingPose(startPose);
        PedroComponent.follower().updatePose();
        telemetryMa.addData("Status:", "inicializado");
        telemetryMa.update(telemetry);
    }

    @Override
    public void onWaitForStart() {
        telemetryMa.addData("READY", "LET'S GO SPACETECH!");
        telemetryMa.update(telemetry);
    }

    @Override
    public void onStartButtonPressed() {
        mecanumDrive.start();
    }

    @Override
    public void onUpdate() {
        PedroComponent.follower().update();
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

        Shooter.INSTANCE.testHood(testHood);

        testTurret.INSTANCE.alignTurretTeleOp(x, y, heading,vx,vy, testTurret.INSTANCE.currentTicks, true);
        telemetryMa.addData("Odometria", "------------------");
        telemetryMa.addData("X", x);
        telemetryMa.addData("Y", y);
        telemetryMa.addData("vX", vx);
        telemetryMa.addData("vY", vy);
        telemetryMa.addData("Heading", Math.toDegrees(heading));
        telemetryMa.addData("Intake", "------------------");
        telemetryMa.addData("Target Intke", Intake.INSTANCE.velocity);
        telemetryMa.addData("Current Intake", Intake.INSTANCE.currentVelocity);
        telemetryMa.addData("Shooter", "------------------");
        telemetryMa.addData("Target Shooter", Shooter.INSTANCE.velocity);
        telemetryMa.addData("Current Shooter", Shooter.INSTANCE.currentVelocity);
        telemetryMa.addData("Distance", testTurret.INSTANCE.movedDistance);
        telemetryMa.update(telemetry);
        BindingManager.update();
    }

    @Override
    public void onStop() {
        BindingManager.reset();
    }
}
