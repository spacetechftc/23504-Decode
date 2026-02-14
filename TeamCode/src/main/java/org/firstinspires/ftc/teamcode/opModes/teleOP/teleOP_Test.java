package org.firstinspires.ftc.teamcode.opModes.teleOP;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mecanismos.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Led;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.Subsystems.testTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "teleOP_Test")
public class teleOP_Test extends NextFTCOpMode {

    public teleOP_Test() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, testTurret.INSTANCE, Led.INSTANCE),
                BindingsComponent.INSTANCE,
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    MecanumDrive mecanumDrive = new MecanumDrive();

    private boolean shooterToggle = false;
    private boolean shooterLastButtonState = false;
    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    @Override
    public void onInit() {
        testTurret.INSTANCE.resetEncoder();
        Intake.INSTANCE.locked.invoke();
        PedroComponent.follower().setStartingPose(startPose);
        PedroComponent.follower().updatePose();
        telemetry.addData("Status:", "inicializado");
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        mecanumDrive.start();
    }

    @Override
    public void onUpdate() {
        double x = PedroComponent.follower().getPose().getX();
        double y = PedroComponent.follower().getPose().getY();
        double heading = PedroComponent.follower().getPose().getHeading();
        double vx = PedroComponent.follower().getVelocity().getXComponent();
        double vy = PedroComponent.follower().getVelocity().getYComponent();
        // Estado do Toggle do shooter
        boolean currentButtonState = gamepad1.right_bumper;

        // Detecta a transição de "solto" -> "pressionado"
        if (currentButtonState && !shooterLastButtonState) {
            shooterToggle = !shooterToggle; // inverte o estado do toggle
        }
        shooterLastButtonState = currentButtonState;

        if (gamepad1.left_trigger > 0.1) {
            Intake.INSTANCE.colet().invoke();
        } else {
            Intake.INSTANCE.stop();
        }
        if (shooterToggle) {
            Intake.INSTANCE.unlocked.invoke();
            Shooter.INSTANCE.shooterOn().invoke();
        } else {
            Intake.INSTANCE.locked.invoke();
            Shooter.INSTANCE.stop();
        }

        testTurret.INSTANCE.alignTurret(x,y,heading,vx,vy, testTurret.INSTANCE.currentTicks, false);

        telemetry.addData("Odometria", "------------------");
        telemetry.addData("X", x);
        telemetry.addData("Y", y);
        telemetry.addData("vX", vx);
        telemetry.addData("vY", vy);
        telemetry.addData("Heading", Math.toDegrees(heading));

        telemetry.addData("Intake", "------------------");
        telemetry.addData("Target Intake", Intake.velocity);
        telemetry.addData("Current Intake", Intake.INSTANCE.currentVelocity);

        telemetry.addData("Shooter", "------------------");
        telemetry.addData("Target Shooter", Shooter.INSTANCE.velocity);
        telemetry.addData("Current Shooter", Shooter.INSTANCE.currentVelocity);
        PedroComponent.follower().update();
        telemetry.update();
    }
}
