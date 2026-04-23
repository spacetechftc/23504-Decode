package org.firstinspires.ftc.teamcode.opModes.Autonomous;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
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

import static org.firstinspires.ftc.teamcode.opModes.Autonomous.Eighteen_Balls_Close_Red.autoEndPoseRed;

@Autonomous(name="21-TwentyOne_Balls_Close_Red", group = "Auto Closes Red", preselectTeleOp = "TeleOp_Red")
public class TwentyOne_Balls_Close_Red extends NextFTCOpMode {

    public TwentyOne_Balls_Close_Red() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, testTurret.INSTANCE, Led.INSTANCE, LimelightSubsystem.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    private TelemetryManager panelsTelemetry;


    // Definição das coordenadas
    private final Pose startPose = new Pose(111, 119, Math.toRadians(39.5));
    private final Pose scorePose = new Pose(74, 71, Math.toRadians(340));

    // Fileira do Meio
    private final Pose prepareMidBalls = new Pose(80, 52, Math.toRadians(0));
    private final Pose takeMidBalls = new Pose(123, 52, Math.toRadians(0));

    // Fileira de Cima
    private final Pose prepareUpBalls = new Pose(88, 74, Math.toRadians(0));
    private final Pose takeUpBalls = new Pose(113, 74, Math.toRadians(0));

    // Gate -- open and colet One
    private final Pose prepareOpenGate = new Pose(114, 49.8, Math.toRadians(38));
    private final Pose openGate = new Pose(127, 49.8, Math.toRadians(38));

    // Gate -- open and colet Two
    private final Pose prepareOpenGateTwo = new Pose(114, 50, Math.toRadians(38));
    private final Pose openGateTwo = new Pose(127, 50, Math.toRadians(38));

    // Leave
    private final Pose leavePose = new Pose(84, 67, Math.toRadians(330));

    private PathChain pathOne, pathTwo, pathThree, pathFour, pathFive;

    private PathChain  pathLeave;

    private PathChain pathGoGate, pathGoBack, pathGoGateTwo, pathGoBackTwo;

    public void buildPaths(Follower follower) {
        pathOne = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setConstantHeadingInterpolation(Math.toRadians(42))
                .build();

        pathTwo = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(scorePose, prepareMidBalls, takeMidBalls))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.01, Intake.INSTANCE.locked)
                .addPath(new BezierLine(takeMidBalls, scorePose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(0.9, Intake.INSTANCE.unlocked)
                .build();

        pathGoGate = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(scorePose, prepareOpenGate, openGate))
                .setLinearHeadingInterpolation(scorePose.getHeading(), openGate.getHeading())
                .addParametricCallback(0.01, Intake.INSTANCE.locked)
                .build();

        pathGoBack = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(openGate, scorePose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(0.9, Intake.INSTANCE.unlocked)
                .build();

        pathGoGateTwo = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(scorePose, prepareOpenGateTwo, openGateTwo))
                .setLinearHeadingInterpolation(scorePose.getHeading(), openGate.getHeading())
                .addParametricCallback(0.01, Intake.INSTANCE.locked)
                .build();

        pathGoBackTwo = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(openGateTwo, scorePose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(0.9, Intake.INSTANCE.unlocked)
                .build();

        pathThree = PedroComponent.follower().pathBuilder()
                .addPath(new BezierCurve(scorePose, prepareUpBalls, takeUpBalls))
                .setTangentHeadingInterpolation()
                .addParametricCallback(0.01, Intake.INSTANCE.locked)
                .addPath(new BezierLine(takeUpBalls, scorePose))
                .setTangentHeadingInterpolation()
                .setReversed()
                .addParametricCallback(0.9, Intake.INSTANCE.unlocked)
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
                    LimelightSubsystem.INSTANCE.artifact.invoke(); Shooter.INSTANCE.switchVelocity(940); Shooter.INSTANCE.switchHood(0.7);
                }),
                new FollowPath(pathOne, true).and(Intake.INSTANCE.unlocked),
                Intake.INSTANCE.coletAutoOn(), new Delay(0.29), // Lançamento
                new FollowPath(pathTwo, true), new Delay(0.29),
                new FollowPath(pathGoGate, true), new Delay(1), new FollowPath(pathGoBack, true), new Delay(0.29),
                new FollowPath(pathGoGate, true), new Delay(1), new FollowPath(pathGoBack, true), new Delay(0.29),
                new FollowPath(pathThree, true), new Delay(0.29),
                new FollowPath(pathGoGateTwo, true), new Delay(1), new FollowPath(pathGoBackTwo, true), new Delay(0.29),
                new FollowPath(pathGoGateTwo, true), new Delay(1), new FollowPath(pathGoBackTwo, true), new Delay(0.29)


        );
    }

    @Override
    public void onInit() {
        LimelightSubsystem.INSTANCE.switchPipeline(4);
        LimelightSubsystem.INSTANCE.setAutonomous(true);
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
        panelsTelemetry.debug("Target Shooter", Shooter.INSTANCE.velocityAuto);
        panelsTelemetry.debug("Current Shooter", Shooter.INSTANCE.currentVelocity);
        panelsTelemetry.update(telemetry);
        Shooter.INSTANCE.shooterAutoOn().invoke();
        testTurret.INSTANCE.alignTurretTeleOp(PedroComponent.follower().getPose().getX(), PedroComponent.follower().getPose().getY(), PedroComponent.follower().getPose().getHeading(),PedroComponent.follower().getVelocity().getXComponent(),PedroComponent.follower().getVelocity().getYComponent(), testTurret.INSTANCE.currentTicks, false);
       // testTurret.INSTANCE.alignTurretAuto(PedroComponent.follower().getPose().getX(), PedroComponent.follower().getPose().getY(), PedroComponent.follower().getPose().getHeading(), testTurret.INSTANCE.currentTicks, false);
        PedroComponent.follower().update();
    }

    @Override
    public void onStop() {
        autoEndPoseRed = PedroComponent.follower().getPose();
    }
}

