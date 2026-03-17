package org.firstinspires.ftc.teamcode.opModes.Autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Led;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.testTurret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name="RoboTech Auto", group = "Auto Closes Red", preselectTeleOp = "TeleOp_Red")
@Configurable
public class RoboTech_Auto extends NextFTCOpMode {
    public RoboTech_Auto() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, testTurret.INSTANCE, Led.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }
    private TelemetryManager panelsTelemetry;
    public static Pose autoEndPoseRed;

    // Definição das coordenadas
    private final Pose startPose = new Pose(111, 114, Math.toRadians(37));
    private final Pose scorePose = new Pose(85, 74, Math.toRadians(325));

    private final Pose curveGate = new Pose(70, 42, Math.toRadians(0));
    private final Pose frontGatePose = new Pose(110, 42, Math.toRadians(24));
    private final Pose gatePose = new Pose(120.5, 42, Math.toRadians(24));

    private final Pose takeBallsUp = new Pose(111, 67.5, Math.toRadians(0));
    private final Pose takeBalls_Mid = new Pose(115, 40.673, Math.toRadians(0));
    private final Pose openGateOne = new Pose(110, 50.5, Math.toRadians(0));
    private final Pose openGateTwo = new Pose(115.5, 50.5, Math.toRadians(0));
    private final Pose takeBalls_Down = new Pose(115, 18.5, Math.toRadians(0));

    private final Pose leave = new Pose(92, 69, Math.toRadians(270));
    private Path pathOne, pathFive, pathSeven, pathLeave, pathScoreOne, pathScoreTwo, pathGate, pathThree, pathOpenGateOne, pathOpenGateTwo;
    private PathChain pathGoGate, pathTwo, pathFour;

    public void buildPaths(Follower follower) {
        pathOne = new Path(new BezierLine(startPose, scorePose));
        pathOne.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        pathTwo = PedroComponent.follower().pathBuilder() // Bola do Meio
                .addPath(
                        new BezierCurve(
                                new Pose(PedroComponent.follower().getPose().getX(), PedroComponent.follower().getPose().getY()),
                                new Pose(60.551, 40.673),
                                takeBalls_Mid
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(0.65, () -> PedroComponent.follower().setMaxPowerScaling(0.8))
                .addParametricCallback(0.97, ()-> PedroComponent.follower().setMaxPowerScaling(1))
                .build();

        pathThree = new Path(new BezierLine(takeBalls_Mid, scorePose));
        pathThree.setLinearHeadingInterpolation(takeBalls_Mid.getHeading(), scorePose.getHeading());

        pathFour = PedroComponent.follower().pathBuilder() // Bolas de Cima
                .addPath(
                        new BezierCurve(
                                new Pose(follower.getPose().getX(), PedroComponent.follower().getPose().getY()),
                                new Pose(54.551,67.5),
                                takeBallsUp
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .addParametricCallback(0.45, ()-> PedroComponent.follower().setMaxPowerScaling(0.7))
                .addParametricCallback(0.97, ()-> PedroComponent.follower().setMaxPowerScaling(1))
                .build();

        pathFive = new Path(new BezierLine(takeBallsUp, scorePose));
        pathFive.setLinearHeadingInterpolation(takeBallsUp.getHeading(), scorePose.getHeading());

        pathSeven = new Path(new BezierLine(takeBalls_Down, scorePose));
        pathSeven.setConstantHeadingInterpolation(Math.toRadians(300));

        pathLeave = new Path(new BezierLine(scorePose, leave));
        pathLeave.setLinearHeadingInterpolation(scorePose.getHeading(), leave.getHeading());

        pathGoGate = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(PedroComponent.follower().getPose().getX(), PedroComponent.follower().getPose().getY()),
                                curveGate,
                                frontGatePose
                        )
                )
                .setConstantHeadingInterpolation(gatePose.getHeading())
                .addParametricCallback(0.76, () -> PedroComponent.follower().setMaxPowerScaling(0.8))
                .addParametricCallback(0.97, () -> PedroComponent.follower().setMaxPowerScaling(1))
                .build();

        pathScoreOne = new Path(new BezierLine(new Pose(PedroComponent.follower().getPose().getX(), PedroComponent.follower().getPose().getY()), new Pose(82, 68)));
        pathScoreOne.setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), scorePose.getHeading());

        pathScoreTwo = new Path(new BezierLine(new Pose(82, 68), scorePose));
        pathScoreTwo.setLinearHeadingInterpolation(scorePose.getHeading(), scorePose.getHeading());

        pathGate = new Path(new BezierLine(frontGatePose, gatePose));
        pathGate.setLinearHeadingInterpolation(frontGatePose.getHeading(), gatePose.getHeading());

        pathOpenGateOne = new Path(new BezierLine(takeBalls_Mid, openGateOne));
        pathOpenGateOne.setConstantHeadingInterpolation(Math.toRadians(0));
        pathOpenGateTwo = new Path(new BezierLine(openGateOne, openGateTwo));
        pathOpenGateTwo.setConstantHeadingInterpolation(Math.toRadians(0));
    }

    public Command autonomousRoutine() {
        return new SequentialGroup(
                new InstantCommand(() -> {
                    Shooter.INSTANCE.switchHood(0.73); Shooter.INSTANCE.switchVelocity(1220);
                }),
                new FollowPath(pathOne, true, 1.0).and(Shooter.INSTANCE.shooterAutoOn()),
                new Delay(0.15), // Torreta se alinhar na primeira vez
                Intake.INSTANCE.coletAutoOn(),
                new Delay(0.6), // Lançamento
                new FollowPath(pathTwo, true, 1.0).and(Intake.INSTANCE.locked),
                new Delay(0.015),
                Intake.INSTANCE.stopAuto(),
                new Delay(0.1),
                new FollowPath(pathOpenGateOne, true, 0.85),
                new FollowPath(pathOpenGateTwo, true, 0.65),
                new Delay(1.6),
                new FollowPath(pathScoreOne, true, 1.0).and(Intake.INSTANCE.unlocked),
                new FollowPath(pathScoreTwo, true, 0.85),
                new Delay(0.3), // Torreta se alinhar
                Intake.INSTANCE.coletAutoOn(),
                new Delay(0.6),
                new FollowPath(pathThree, true, 0.9).and(Intake.INSTANCE.unlocked),
                new Delay(0.2), // Torreta se alinhar na segunda vez
                Intake.INSTANCE.coletAutoOn(),
                new Delay(0.6), // Lançamento
                new FollowPath(pathGoGate, true, 1.0).and(Intake.INSTANCE.locked),
                new FollowPath(pathGate, true, 0.65).and(Intake.INSTANCE.locked),
                new Delay(1.7), // Espera no gate
                Intake.INSTANCE.stopAuto(),
                new Delay(0.1),
                new FollowPath(pathScoreOne, true, 1.0).and(Intake.INSTANCE.unlocked),
                new FollowPath(pathScoreTwo, true, 0.85),
                new Delay(0.1), // Torreta se alinhar
                Intake.INSTANCE.coletAutoOn(),
                new Delay(0.6),
                new FollowPath(pathFour, true, 1.0).and(Intake.INSTANCE.locked),
                new Delay(0.1),
                Intake.INSTANCE.stopAuto(),
                new Delay(0.1),
                new FollowPath(pathFive, true, 0.8).and(Intake.INSTANCE.unlocked),
                new Delay(0.3), // Torreta se alinhar
                Intake.INSTANCE.coletAutoOn(),
                new Delay(0.6), // Lançamento
                new FollowPath(pathLeave)


        );
    }


    @Override
    public void onInit() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        PedroComponent.follower().setStartingPose(startPose);

        testTurret.INSTANCE.resetEncoder();
        PedroComponent.follower().updatePose();
        buildPaths(PedroComponent.follower());

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void onWaitForStart() {
        testTurret.INSTANCE.resetEncoder();
    }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }

    @Override
    public void onUpdate() {
        panelsTelemetry.debug("X", PedroComponent.follower().getPose().getX());
        panelsTelemetry.debug("Y", PedroComponent.follower().getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));
        panelsTelemetry.debug("Shooter", "------------------");
        panelsTelemetry.debug("Target Shooter", Shooter.INSTANCE.velocityAuto);
        panelsTelemetry.debug("Current Shooter", Shooter.INSTANCE.currentVelocity);
        panelsTelemetry.update(telemetry);
        testTurret.INSTANCE.alignTurretAuto(PedroComponent.follower().getPose().getX(), PedroComponent.follower().getPose().getY(), PedroComponent.follower().getPose().getHeading(), testTurret.INSTANCE.currentTicks, false);
        telemetry.update();
    }

    @Override
    public void onStop() {
        autoEndPoseRed = PedroComponent.follower().getPose();
    }
}

