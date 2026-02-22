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

import static org.firstinspires.ftc.teamcode.opModes.Autonomous.Eighteen_Balls_Close_Red.autoEndPoseRed;

@Autonomous(name="15 Balls Close Red", group = "Auto Closes Red", preselectTeleOp = "TeleOp_Red")
@Configurable
public class Fifteen_Balls_Close_Red extends NextFTCOpMode {
    public Fifteen_Balls_Close_Red() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, testTurret.INSTANCE, Led.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }
    private TelemetryManager panelsTelemetry;

    // Definição das coordenadas
    private final Pose startPose = new Pose(111, 114, Math.toRadians(37));
    private final Pose scorePose = new Pose(85, 74, Math.toRadians(325));

    private final Pose curveGate = new Pose(70, 42.5, Math.toRadians(0));

    private final Pose frontGatePose = new Pose(110, 42.5, Math.toRadians(20));
    private final Pose gatePose = new Pose(122, 42.5, Math.toRadians(20));

    private final Pose takeBallsUp = new Pose(116, 66.673, Math.toRadians(0));
    private final Pose takeBalls_Mid = new Pose(118.626, 40.673, Math.toRadians(0));

    private final Pose leave = new Pose(90, 60, Math.toRadians(270));

    private Follower follower;
    private Path pathOne, pathThree, pathFive, pathLeave, pathScoreOne, pathScoreTwo, pathGate;
    private PathChain pathGoGate, pathTwo, pathFour;

    public void buildPaths() {
        pathOne = new Path(new BezierLine(startPose, scorePose));
        pathOne.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        pathTwo = follower.pathBuilder() // Bola do Meio
                .addPath(
                        new BezierCurve(
                                new Pose(follower.getPose().getX(), follower.getPose().getY()),
                                new Pose(54.551, 40.673),
                                takeBalls_Mid
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(0))
                .build();

        pathThree = new Path(new BezierLine(takeBalls_Mid, scorePose));
        pathThree.setLinearHeadingInterpolation(takeBalls_Mid.getHeading(), scorePose.getHeading());

       pathFour = follower.pathBuilder() // Bolas de Cima
               .addPath(
                       new BezierCurve(
                               new Pose(follower.getPose().getX(), follower.getPose().getY()),
                               new Pose(54.551,66.6973),
                               takeBallsUp
                       )
               )
               .setConstantHeadingInterpolation(Math.toRadians(0))
               .build();

       pathFive = new Path(new BezierLine(takeBallsUp, scorePose));
       pathFive.setLinearHeadingInterpolation(takeBallsUp.getHeading(), scorePose.getHeading());

        pathLeave = new Path(new BezierLine(scorePose, leave));
        pathLeave.setLinearHeadingInterpolation(scorePose.getHeading(), leave.getHeading());

        pathGoGate = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(follower.getPose().getX(), follower.getPose().getY()),
                                curveGate,
                                frontGatePose
                        )
                )
                .setConstantHeadingInterpolation(gatePose.getHeading())
                .build();

        pathScoreOne = new Path(new BezierLine(new Pose(follower.getPose().getX(), follower.getPose().getY()), new Pose(82, 70)));
        pathScoreOne.setLinearHeadingInterpolation(follower.getHeading(), scorePose.getHeading());

        pathScoreTwo = new Path(new BezierLine(new Pose(82, 70), scorePose));
        pathScoreTwo.setLinearHeadingInterpolation(scorePose.getHeading(), scorePose.getHeading());

        pathGate = new Path(new BezierLine(frontGatePose, gatePose));
        pathGate.setLinearHeadingInterpolation(frontGatePose.getHeading(), gatePose.getHeading());
    }

    public Command autonomousRoutine() {
        return new SequentialGroup(
                new InstantCommand(() -> {
                    Shooter.INSTANCE.switchHood(0.72); Shooter.INSTANCE.switchVelocity(1200);
                }),
                new FollowPath(pathOne, true, 1.0).and(Shooter.INSTANCE.shooterAutoOn()),
                new Delay(0.02), // Torreta se alinhar na primeira vez
                Intake.INSTANCE.coletAutoOn(),
                new Delay(0.5), // Lançamento
                new FollowPath(pathTwo, true, 1.0).and(Intake.INSTANCE.locked),
                new Delay(0.15), // Pegou a fileira do meio
                Intake.INSTANCE.stopAuto(),
                new Delay(0.05), // Certeza que desligou o intake
                new FollowPath(pathThree, true, 1.0).and(Intake.INSTANCE.unlocked),
                new Delay(0.015), // Torreta se alinhar na segunda vez
                Intake.INSTANCE.coletAutoOn(),
                new Delay(0.5), // Lançamento
                new FollowPath(pathGoGate, true, 1.0).and(Intake.INSTANCE.locked),
                new FollowPath(pathGate, true, 0.8).and(Intake.INSTANCE.locked),
                new Delay(1.8), // Espera no gate
                Intake.INSTANCE.stopAuto(),
                new Delay(0.05),
                new FollowPath(pathScoreOne, true, 1.0).and(Intake.INSTANCE.unlocked),
                new FollowPath(pathScoreTwo),
                new Delay(0.015), // Torreta se alinhar
                Intake.INSTANCE.coletAutoOn(),
                new Delay(0.5),
                new FollowPath(pathGoGate, true, 1.0).and(Intake.INSTANCE.locked),
                new FollowPath(pathGate, true, 0.8).and(Intake.INSTANCE.locked),
                new Delay(1.8), // Espera no gate
                Intake.INSTANCE.stopAuto(),
                new Delay(0.05),
                new FollowPath(pathScoreOne, true, 1.0).and(Intake.INSTANCE.unlocked),
                new FollowPath(pathScoreTwo),
                new Delay(0.015), // Torreta se alinhar
                Intake.INSTANCE.coletAutoOn(),
                new Delay(0.5),
                new FollowPath(pathFour, true, 1.0).and(Intake.INSTANCE.locked),
                new Delay(0.15), // Fileira de Cima certeza que pegou
                Intake.INSTANCE.stopAuto(),
                new Delay(0.05), // Certeza que desligou o intake
                new FollowPath(pathFive, true, 1.0).and(Intake.INSTANCE.unlocked),
                new Delay(0.015), // Torreta se alinhar
                Intake.INSTANCE.coletAutoOn(),
                new Delay(0.5), // Lançamento
                new FollowPath(pathLeave, true),
                new InstantCommand(() -> {
                    Shooter.INSTANCE.switchHood(0.51); Shooter.INSTANCE.stopAuto();
                }),
                Intake.INSTANCE.stopAuto()
        );
    }


    @Override
    public void onInit() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);

        testTurret.INSTANCE.resetEncoder();
        follower.updatePose();
        buildPaths();

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
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(follower.getPose().getHeading()));
        testTurret.INSTANCE.alignTurretAuto(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), testTurret.INSTANCE.currentTicks, false);
        panelsTelemetry.update(telemetry);
        telemetry.update();
        follower.update();
    }

    @Override
    public void onStop() {
        autoEndPoseRed = follower.getPose();
    }
}

