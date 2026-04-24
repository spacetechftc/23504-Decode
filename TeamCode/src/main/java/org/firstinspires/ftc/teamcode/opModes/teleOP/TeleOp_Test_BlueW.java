package org.firstinspires.ftc.teamcode.opModes.teleOP;

import static org.firstinspires.ftc.teamcode.Subsystems.Led.PrismState.GREEN_SNAKE;
import static org.firstinspires.ftc.teamcode.Subsystems.Led.PrismState.GREEN_SOLID;
import static org.firstinspires.ftc.teamcode.Subsystems.Led.PrismState.RED_FILL;
import static org.firstinspires.ftc.teamcode.Subsystems.Led.PrismState.RED_SNAKE;
import static org.firstinspires.ftc.teamcode.Subsystems.Led.PrismState.RED_SOLID;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.skeletonarmy.marrow.TimerEx;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.teamcode.Mecanismos.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.EndGame;
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
@TeleOp(name = "TeleOp_Test_Blue_Worlds", group = "TeleOp Tests")
public class TeleOp_Test_BlueW extends NextFTCOpMode {

    public TeleOp_Test_BlueW() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, testTurret.INSTANCE, LimelightSubsystem.INSTANCE, Led.INSTANCE),
                BindingsComponent.INSTANCE,
                BulkReadComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    private final PolygonZone closeLaunchZone = new PolygonZone(new Point(134, 134), new Point(68, 68), new Point(0, 134));
    private final PolygonZone blueBase = new PolygonZone(new Point(98, 26), 20, 20);
    private final PolygonZone robotZone = new PolygonZone(16, 14);
    private final PolygonZone farLaunchZone = new PolygonZone(new Point(48.5, 0), new Point(63, 14), new Point(77, 0));
    MecanumDrive mecanumDrive = new MecanumDrive();
    private final Pose startPose = new Pose(127, -1.45, Math.toRadians(180));
    private final Pose resetPose = new Pose(127, -1.45, Math.toRadians(180));

    public static TelemetryManager telemetryMa;

    private boolean turretToggle = false;
    private boolean turretLastButtonState = false;

    private boolean endGameToggle = false;
    private boolean endGameLastButtonState = false;

    private int count = 0;

    TimerEx teleOpTime = new TimerEx(120);
    @Override
    public void onInit() {
        telemetryMa = PanelsTelemetry.INSTANCE.getTelemetry();
        testTurret.INSTANCE.resetEncoder();
        LimelightSubsystem.INSTANCE.switchPipeline(1);
        LimelightSubsystem.INSTANCE.setTeleOp(true);
        PedroComponent.follower().setStartingPose(startPose);
        PedroComponent.follower().updatePose();
        telemetryMa.addData("Status:", "inicializado");
        telemetryMa.update(telemetry);
    }

    @Override
    public void onStartButtonPressed() {
        teleOpTime.start();
        mecanumDrive.start();
    }

    @Override
    public void onUpdate() {
        double x = PedroComponent.follower().getPose().getX();
        double y = PedroComponent.follower().getPose().getY();
        double heading = PedroComponent.follower().getPose().getHeading();
        double vx = PedroComponent.follower().getVelocity().getXComponent();
        double vy = PedroComponent.follower().getVelocity().getYComponent();
        robotZone.setPosition(PedroComponent.follower().getPose().getX(), PedroComponent.follower().getPose().getY());
        robotZone.setRotation(PedroComponent.follower().getPose().getHeading());

        if (teleOpTime.isLessThan(11)) {
            Led.INSTANCE.setPrismState(
                    robotZone.isInside(blueBase) ? RED_SNAKE : RED_FILL
            );

        } else if (teleOpTime.isLessThan(21)) {
            Led.INSTANCE.setPrismState(
                    robotZone.isInside(blueBase) ? RED_SNAKE : RED_SOLID
            );

        } else {
            Led.INSTANCE.setPrismState(
                    (robotZone.isInside(closeLaunchZone) || robotZone.isInside(farLaunchZone))
                            ? GREEN_SNAKE
                            : GREEN_SOLID
            );
        }

        boolean currentButtonState = gamepad1.right_bumper;
        if (currentButtonState && !turretLastButtonState) {
            turretToggle = !turretToggle;
        }
        turretLastButtonState = currentButtonState;

        //Toggle EndGame
        boolean endGameCurrentButtonState = gamepad1.dpad_down;
        if (endGameCurrentButtonState && !endGameLastButtonState) {
            endGameToggle = !endGameToggle;
        }
        endGameLastButtonState = endGameCurrentButtonState;

        if (gamepad1.a) {
            PedroComponent.follower().setPose(new Pose(LimelightSubsystem.INSTANCE.pedroY, LimelightSubsystem.INSTANCE.pedroX, LimelightSubsystem.INSTANCE.headingRadians));
        }

        if (gamepad1.right_stick_button) {
            PedroComponent.follower().setPose(resetPose);
        }

        if (count == 0) {
            LimelightSubsystem.INSTANCE.relocalization.invoke();
        }

        if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
            if (gamepad1.right_trigger > 0.1) {
                Intake.INSTANCE.unlocked.invoke();
            } else {
                Intake.INSTANCE.locked.invoke();
            }
            Intake.INSTANCE.colet().invoke();
        } else {
            Intake.INSTANCE.stop();
            Intake.INSTANCE.locked.invoke();
        }

        if (turretToggle) {
            testTurret.INSTANCE.turretToPosition(0);
        } else {
            testTurret.INSTANCE.alignTurretTeleOp(x, y, heading,vx,vy, testTurret.INSTANCE.currentTicks, true);
        }
/*
        if (endGameToggle) {
            EndGame.INSTANCE.endGameOn();
        } else {
            EndGame.INSTANCE.endGameOff();
        }

 */

        Shooter.INSTANCE.initMechanisms(true);
        Shooter.INSTANCE.shooterOn().invoke();

        telemetryMa.addData("Odometria", "------------------");
        telemetryMa.addData("X", x);
        telemetryMa.addData("Y", y);
        telemetryMa.addData("Heading", Math.toDegrees(heading));
        telemetryMa.addData("Target Shooter", Shooter.INSTANCE.velocity);
        telemetryMa.addData("Current Shooter", Shooter.INSTANCE.currentVelocity);
        telemetryMa.addData("current Intake", Intake.INSTANCE.currentVelocity);
        telemetryMa.update(telemetry);
        PedroComponent.follower().update();
    }
    @Override
    public void onStop() {
        EndGame.INSTANCE.endGameOff();
    }
}