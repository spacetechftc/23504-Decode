package org.firstinspires.ftc.teamcode.opModes.teleOP;

import static org.firstinspires.ftc.teamcode.opModes.Autonomous.Eighteen_Balls_Close_Red.autoEndPoseRed;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mecanismos.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Led;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.testTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name = "TeleOp_Red", group = "Official")
public class TeleOp_Red extends NextFTCOpMode {

    public TeleOp_Red() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, testTurret.INSTANCE, Led.INSTANCE, LimelightSubsystem.INSTANCE),
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

    public static Pose resetPose = new Pose(0,0,Math.toRadians(0));

    @Override
    public void onInit() {
        PedroComponent.follower().setStartingPose(autoEndPoseRed == null ? new Pose() : autoEndPoseRed);
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
        PedroComponent.follower().update();
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
            testTurret.INSTANCE.alignTurretTeleOp(x, y, heading,vx,vy, testTurret.INSTANCE.currentTicks, false);
            Shooter.INSTANCE.initMechanisms(true);
        }

        if (gamepad2.dpad_up) {
            Pose recoveryPose = LimelightSubsystem.getRobotPoseMT1();
            if (recoveryPose != null) {
                PedroComponent.follower().setPose(recoveryPose);
            }
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
        telemetry.update();
    }
}
