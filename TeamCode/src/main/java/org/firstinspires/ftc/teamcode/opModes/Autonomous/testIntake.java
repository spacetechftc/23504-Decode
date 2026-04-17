package org.firstinspires.ftc.teamcode.opModes.Autonomous;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Led;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
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

@Autonomous(name="testIntake", group = "Auto Closes Red", preselectTeleOp = "TeleOp_Red")
public class testIntake extends NextFTCOpMode {

    public testIntake() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, testTurret.INSTANCE, Led.INSTANCE, LimelightSubsystem.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    private TelemetryManager panelsTelemetry;
    public static Pose autoEndPoseRed;


    // Definição das coordenadas
    private final Pose startPose = new Pose(110, 118, Math.toRadians(39.5));
    private final Pose scorePose = new Pose(72, 71, Math.toRadians(340));

    // Fileira do Meio
    private final Pose prepareMidBalls = new Pose(88, 52, Math.toRadians(0));
    private final Pose takeMidBalls = new Pose(128, 52, Math.toRadians(0));

    // Fileira de Cima
    private final Pose prepareUpBalls = new Pose(88, 74, Math.toRadians(0));
    private final Pose takeUpBalls = new Pose(120, 74, Math.toRadians(0));

    // Gate -- open and colet
    private final Pose openGate = new Pose(124.5, 48.5, Math.toRadians(32));

    // Leave
    private final Pose leavePose = new Pose(84, 67, Math.toRadians(330));

    private PathChain pathOne, pathTwo, pathThree, pathFour, pathFive;

    private PathChain  pathLeave;

    private PathChain pathGoGate, pathGoBack;

    public void buildPaths(Follower follower) {
        pathOne = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setConstantHeadingInterpolation(Math.toRadians(42))
                .build();

        pathTwo = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(scorePose, prepareMidBalls, takeMidBalls))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.01, Intake.INSTANCE.locked)
                .build();

        pathThree = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(takeMidBalls, scorePose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        pathGoGate = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scorePose, openGate))
                .setLinearHeadingInterpolation(scorePose.getHeading(), openGate.getHeading())
                .addParametricCallback(0.01, Intake.INSTANCE.locked)
                .build();

        pathGoBack = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(openGate, scorePose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .build();

        pathFour = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(scorePose, prepareUpBalls, takeUpBalls))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.01, Intake.INSTANCE.locked)
                .build();

        pathFive = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(takeUpBalls, scorePose))
                .setLinearHeadingInterpolation(takeUpBalls.getHeading(), scorePose.getHeading())
                .build();

        pathLeave = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scorePose, leavePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leavePose.getHeading())
                .addParametricCallback(0.01, Intake.INSTANCE.locked)
                .build();
    }

    public Command autonomousRoutine() {
        return new SequentialGroup(
                new InstantCommand(() -> {
                    LimelightSubsystem.INSTANCE.artifact.invoke(); Shooter.INSTANCE.switchVelocity(960); Shooter.INSTANCE.switchHood(0.65);
                }),
                new FollowPath(pathOne, true).and(Intake.INSTANCE.unlocked),
                new Delay(0.01), Intake.INSTANCE.coletAutoOn(), new Delay(0.35),
                // Lançamento
                new FollowPath(pathTwo, true), new FollowPath(pathThree, true),
                new Delay(0.004), Intake.INSTANCE.unlocked, new Delay(0.35),// Lançamento
                new FollowPath(pathGoGate, true), new Delay(1.2), new FollowPath(pathGoBack, true),
                new Delay(0.004), Intake.INSTANCE.unlocked, new Delay(0.35), // Lançamento
                new FollowPath(pathGoGate, true), new Delay(1.2), new FollowPath(pathGoBack, true),
                new Delay(0.004), Intake.INSTANCE.unlocked, new Delay(0.35), // Lançamento
                new FollowPath(pathFour, true), new FollowPath(pathFive, true),
                new Delay(0.004), Intake.INSTANCE.unlocked, new Delay(0.35),
                new FollowPath(pathGoGate, true), new Delay(1.2), new FollowPath(pathGoBack, true),
                new Delay(0.004), Intake.INSTANCE.unlocked, new Delay(0.35) // Lançamento
        );
    }

    @Override
    public void onInit() {
        LimelightSubsystem.INSTANCE.switchPipeline(4);
        LimelightSubsystem.INSTANCE.setTeleOp(true);
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
        panelsTelemetry.debug("Current Pose X", PedroComponent.follower().getPose().getX());
        panelsTelemetry.debug("Current Pose Y", PedroComponent.follower().getPose().getY());
        panelsTelemetry.debug("Current Pose Heading", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));
        panelsTelemetry.update(telemetry);
        Shooter.INSTANCE.shooterAutoOn().invoke();
        //testTurret.INSTANCE.alignTurretTeleOp(PedroComponent.follower().getPose().getX(), PedroComponent.follower().getPose().getY(), PedroComponent.follower().getPose().getHeading(),PedroComponent.follower().getVelocity().getXComponent(),PedroComponent.follower().getVelocity().getYComponent(), testTurret.INSTANCE.currentTicks, false);
        testTurret.INSTANCE.alignTurretAuto(PedroComponent.follower().getPose().getX(), PedroComponent.follower().getPose().getY(), PedroComponent.follower().getPose().getHeading(), testTurret.INSTANCE.currentTicks, false);
        PedroComponent.follower().update();
    }

    @Override
    public void onStop() {
        autoEndPoseRed = PedroComponent.follower().getPose();
    }
}

