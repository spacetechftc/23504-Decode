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
import dev.nextftc.core.commands.groups.ParallelDeadlineGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name="18 Balls Close Blue", group = "Auto Closes Blue", preselectTeleOp = "TeleOp_Blue")
@Configurable
public class Eighteen_Balls_Close_Blue extends NextFTCOpMode {
    public Eighteen_Balls_Close_Blue() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, testTurret.INSTANCE, Led.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }
    private TelemetryManager panelsTelemetry;
    public static Pose autoEndPoseBlue;

    // Definição das coordenadas
    private final Pose startPose = new Pose(20, 114, Math.toRadians(139));
    private final Pose scorePose = new Pose(49.5, 70, Math.toRadians(220));
    private final Pose curveGate = new Pose(60, 45.5, Math.toRadians(180));
    private final Pose frontGatePose = new Pose(18, 45.5, Math.toRadians(150));
    private final Pose gatePose = new Pose(2.5, 45.5, Math.toRadians(150));
    private final Pose takeBallsUp = new Pose(24, 67, Math.toRadians(180));
    private final Pose takeBalls_Mid = new Pose(16, 40, Math.toRadians(180));
    private final Pose takeBalls_Down = new Pose(16, 18, Math.toRadians(180));

    private Path pathOne, pathThree, pathFive, pathSeven, pathScoreOne, pathScoreTwo, pathGate;
    private PathChain pathGoGate, pathTwo, pathFour, pathSix;

    public void buildPaths(Follower follower) {
        pathOne = new Path(new BezierLine(startPose, scorePose));
        pathOne.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        pathTwo = PedroComponent.follower().pathBuilder() // Bola do Meio
                .addPath(
                        new BezierCurve(
                                new Pose(PedroComponent.follower().getPose().getX(), PedroComponent.follower().getPose().getY()),
                                new Pose(65, 40),
                                takeBalls_Mid
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.65, () -> PedroComponent.follower().setMaxPowerScaling(0.7))
                .addParametricCallback(0.97, ()-> PedroComponent.follower().setMaxPowerScaling(1))
                .build();

        pathThree = new Path(new BezierLine(takeBalls_Mid, scorePose));
        pathThree.setLinearHeadingInterpolation(takeBalls_Mid.getHeading(), scorePose.getHeading());


        pathFour = PedroComponent.follower().pathBuilder() // Bolas de Cima
                .addPath(
                        new BezierCurve(
                                new Pose(follower.getPose().getX(), PedroComponent.follower().getPose().getY()),
                                new Pose(60,67),
                                takeBallsUp
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.45, ()-> PedroComponent.follower().setMaxPowerScaling(0.7))
                .addParametricCallback(0.97, ()-> PedroComponent.follower().setMaxPowerScaling(1))
                .build();

        pathFive = new Path(new BezierLine(takeBallsUp, scorePose));
        pathFive.setLinearHeadingInterpolation(takeBallsUp.getHeading(), scorePose.getHeading());

        pathSix = PedroComponent.follower().pathBuilder() // Bola de Baixo
                .addPath(
                        new BezierCurve(
                                new Pose(PedroComponent.follower().getPose().getX(), PedroComponent.follower().getPose().getY()),
                                new Pose(63.551,18),
                                takeBalls_Down
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.7, ()-> PedroComponent.follower().setMaxPowerScaling(0.8))
                .addParametricCallback(0.97, ()-> PedroComponent.follower().setMaxPowerScaling(1))
                .build();

        pathSeven = new Path(new BezierLine(takeBalls_Down, scorePose));
        pathSeven.setConstantHeadingInterpolation(Math.toRadians(240));

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

        pathScoreOne = new Path(new BezierLine(new Pose(PedroComponent.follower().getPose().getX(), PedroComponent.follower().getPose().getY()), new Pose(47, 71)));
        pathScoreOne.setLinearHeadingInterpolation(PedroComponent.follower().getHeading(), scorePose.getHeading());

        pathScoreTwo = new Path(new BezierLine(new Pose(47, 71), scorePose));
        pathScoreTwo.setLinearHeadingInterpolation(scorePose.getHeading(), scorePose.getHeading());

        pathGate = new Path(new BezierLine(frontGatePose, gatePose));
        pathGate.setLinearHeadingInterpolation(frontGatePose.getHeading(), gatePose.getHeading());
    }

    public Command autonomousRoutine() {
        return new SequentialGroup(
                new InstantCommand(() -> {
                    Shooter.INSTANCE.switchHood(0.84); Shooter.INSTANCE.switchVelocity(1280);
                }),
                new FollowPath(pathOne, true, 1.0).and(Shooter.INSTANCE.shooterAutoOn()),
                new Delay(0.05), // Torreta se alinhar na primeira vez
                Intake.INSTANCE.coletAutoOn(),
                new Delay(0.4), // Lançamento
                new FollowPath(pathTwo, true, 1.0).and(Intake.INSTANCE.locked),
                new Delay(0.02),
                Intake.INSTANCE.stopAuto(),
                new Delay(0.03), // Certeza que desligou o intake
                new FollowPath(pathThree, true, 0.9).and(Intake.INSTANCE.unlocked),
                new Delay(0.035), // Torreta se alinhar na segunda vez
                Intake.INSTANCE.coletAutoOn(),
                new Delay(0.4), // Lançamento
                new FollowPath(pathGoGate, true, 1.0).and(Intake.INSTANCE.locked),
                new FollowPath(pathGate, true, 0.65).and(Intake.INSTANCE.locked),
                new Delay(1), // Espera no gate
                Intake.INSTANCE.stopAuto(),
                new Delay(0.02),
                new FollowPath(pathScoreOne, true, 1.0).and(Intake.INSTANCE.unlocked),
                new FollowPath(pathScoreTwo, true, 0.9),
                new Delay(0.02),
                Intake.INSTANCE.coletAutoOn(),
                new Delay(0.4),
                new FollowPath(pathGoGate, true, 1.0).and(Intake.INSTANCE.locked),
                new FollowPath(pathGate, true, 0.65).and(Intake.INSTANCE.locked),
                new Delay(1), // Espera no gate
                Intake.INSTANCE.stopAuto(),
                new Delay(0.02),
                new FollowPath(pathScoreOne, true, 1.0).and(Intake.INSTANCE.unlocked),
                new FollowPath(pathScoreTwo, true, 0.9),
                new Delay(0.02),
                Intake.INSTANCE.coletAutoOn(),
                new Delay(0.4),
                new FollowPath(pathFour, true, 1.0).and(Intake.INSTANCE.locked),
                new Delay(0.1),
                Intake.INSTANCE.stopAuto(),
                new Delay(0.02), // Certeza que desligou o intake
                new FollowPath(pathFive, true, 0.8).and(Intake.INSTANCE.unlocked),
                new Delay(0.08), // Torreta se alinhar
                Intake.INSTANCE.coletAutoOn(),
                new Delay(0.4), // Lançamento
                new FollowPath(pathSix, true, 1.0).and(Intake.INSTANCE.locked),
                new Delay(0.035),
                Intake.INSTANCE.stopAuto(),
                new Delay(0.02),
                new FollowPath(pathSeven, true, 0.7).and(Intake.INSTANCE.unlocked),
                new Delay(0.15),
                Intake.INSTANCE.coletAutoOn(),
                new Delay(0.45)
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
        testTurret.INSTANCE.alignTurretAuto(PedroComponent.follower().getPose().getX(), PedroComponent.follower().getPose().getY(), PedroComponent.follower().getPose().getHeading(), testTurret.INSTANCE.currentTicks, true);
        telemetry.update();
    }

    @Override
    public void onStop() {
        autoEndPoseBlue = PedroComponent.follower().getPose();
    }
}

