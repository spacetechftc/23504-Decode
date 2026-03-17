package org.firstinspires.ftc.teamcode.opModes.teleOP;

import static org.firstinspires.ftc.teamcode.opModes.Autonomous.Eighteen_Balls_Close_Blue.autoEndPoseBlue;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mecanismos.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Led;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.testTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "TeleOp_Blue", group = "Official")
public class TeleOp_Blue extends NextFTCOpMode {

    public TeleOp_Blue() {
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

    private boolean turretToggle = false;
    private boolean turretLastButtonState = false;

    public static Pose resetPose = new Pose(128.4,-2.1,Math.toRadians(180));

    @Override
    public void onInit() {
        PedroComponent.follower().setStartingPose(autoEndPoseBlue == null ? new Pose() : autoEndPoseBlue);
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

        boolean currentButtonStateSquare = gamepad2.square;
        if (currentButtonStateSquare && !turretLastButtonState) {
            turretToggle = !turretToggle; // inverte o estado do toggle
        }
        turretLastButtonState = currentButtonStateSquare;

        if (gamepad1.left_trigger > 0.1) {
            Intake.INSTANCE.colet().invoke();
        } else {
            Intake.INSTANCE.stop();
        }

        if (shooterToggle) {
            if (turretToggle) {
                Intake.INSTANCE.unlocked.invoke();
                Shooter.INSTANCE.fixedVelocity().invoke();
            } else {
                Intake.INSTANCE.unlocked.invoke();
                Shooter.INSTANCE.shooterOn().invoke();
            }
        } else {
            Intake.INSTANCE.locked.invoke();
            Shooter.INSTANCE.stopTeleOp();
        }

        if (gamepad2.right_stick_button) {
            PedroComponent.follower().setPose(resetPose);
        }

        if (turretToggle) {
            testTurret.INSTANCE.turretToPosition(0);
            Shooter.INSTANCE.switchHood(0.51);
        } else {
            testTurret.INSTANCE.alignTurretTeleOp(x, y, heading,vx,vy, testTurret.INSTANCE.currentTicks, true);
            Shooter.INSTANCE.initMechanisms(false);
        }

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
