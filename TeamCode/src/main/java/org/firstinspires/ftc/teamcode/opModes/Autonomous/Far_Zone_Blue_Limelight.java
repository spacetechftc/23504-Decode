package org.firstinspires.ftc.teamcode.opModes.Autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Mecanismos.MecanumDrive;
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
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

import static org.firstinspires.ftc.teamcode.opModes.Autonomous.Eighteen_Balls_Close_Blue.autoEndPoseBlue;

@Autonomous(name="Far_Zone_Blue_Limelight", group = "Auto Far Blue", preselectTeleOp = "TeleOp_Blue")
@Configurable
public class Far_Zone_Blue_Limelight extends NextFTCOpMode {

    public Far_Zone_Blue_Limelight() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, testTurret.INSTANCE, Led.INSTANCE, LimelightSubsystem.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    private TelemetryManager panelsTelemetry;
    private final Pose startPose = new Pose(46, -0.1, Math.toRadians(180));
    private final Pose scorePose = new Pose(50, 11, Math.toRadians(180));
    private final Pose leavePose = new Pose(30, 13, Math.toRadians(180));

    public int pos;

    private PathChain pathOne, pathTwo, pathThree, pathFour, pathFive, pathSeven, pathSix, pathLeave;

    // Fileira de Baixo
    private final Pose prepareDownBalls = new Pose(35, 27, Math.toRadians(180));
    private final Pose takeDownBalls = new Pose(-3, 27, Math.toRadians(180));
    // Three Balls
    private final Pose firstThreeBalls = new Pose(1.2, 10, Math.toRadians(180));
    private final Pose backThreeBalls = new Pose(32, 5, Math.toRadians(180));
    private final Pose secondThreeBalls = new Pose(1.2, 0.5, Math.toRadians(180));

    private final Pose secondTrajectory = new Pose(0, 30, Math.toRadians(180));



    // Variáveis do closed loop
    private int alignedCycles = 0;
    private double lastLateralOffset = 0;

    private boolean startedAlignPath = false;
    private long targetDetectedTime = 0;
    private boolean waitingDelay = false;

    private final ElapsedTime autoTimer = new ElapsedTime();
    private boolean abortToLeave = false;
    private boolean forcedLeave = false;
    private boolean leavePathStarted = false;

    public void buildPaths(Follower follower) {
        pathOne = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();

        pathTwo = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scorePose, prepareDownBalls))
                .setLinearHeadingInterpolation(scorePose.getHeading(), prepareDownBalls.getHeading())
                .addParametricCallback(0.01, Intake.INSTANCE.locked)
                .addPath(new BezierLine(prepareDownBalls, takeDownBalls))
                .setLinearHeadingInterpolation(prepareDownBalls.getHeading(), takeDownBalls.getHeading())
                .build();

        pathThree = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(takeDownBalls, scorePose))
                .setLinearHeadingInterpolation(takeDownBalls.getHeading(), scorePose.getHeading())
                .build();

        pathFour = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scorePose, firstThreeBalls))
                .setLinearHeadingInterpolation(scorePose.getHeading(), firstThreeBalls.getHeading())
                .setConstraints(new PathConstraints(0.5, 100))
                .addParametricCallback(0.01, Intake.INSTANCE.locked)
                .build();

        pathFive = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(firstThreeBalls, backThreeBalls))
                .setLinearHeadingInterpolation(firstThreeBalls.getHeading(), backThreeBalls.getHeading())
                .setConstraints(new PathConstraints(0.5, 100))
                .build();

        pathSix = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(backThreeBalls, secondThreeBalls))
                .setLinearHeadingInterpolation(backThreeBalls.getHeading(), secondThreeBalls.getHeading())
                .build();

        pathSeven = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scorePose, secondTrajectory))
                .setConstraints(new PathConstraints(0.5, 100))
                .setLinearHeadingInterpolation(scorePose.getHeading(), secondTrajectory.getHeading())
                .addParametricCallback(0.01, Intake.INSTANCE.locked)
                .build();

        pathLeave = PedroComponent.follower().pathBuilder()
                .addPath(new BezierLine(scorePose, leavePose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), leavePose.getHeading())
                .build();
    }

    public Command alignToBallsSingleCorrection() {
        return new LambdaCommand()
                .setStart(() -> {
                    if (abortToLeave) return;
                    LimelightSubsystem.INSTANCE.artifact.invoke();
                    startedAlignPath = false;
                    waitingDelay = false;
                    targetDetectedTime = 0;
                })
                .setUpdate(() -> {
                    if (abortToLeave) return;

                    if (!LimelightSubsystem.INSTANCE.hasTarget()) {
                        waitingDelay = false;
                        targetDetectedTime = 0;
                        return;
                    }

                    if (!waitingDelay) {
                        targetDetectedTime = System.currentTimeMillis();
                        waitingDelay = true;
                        return;
                    }

                    if (System.currentTimeMillis() - targetDetectedTime < 250) {
                        return;
                    }

                    if (startedAlignPath || PedroComponent.follower().isBusy()) {
                        return;
                    }

                    Pose current = PedroComponent.follower().getPose();
                    double tx = LimelightSubsystem.INSTANCE.tx;

                    double lateralOffset = tx * 1.05;
                    lateralOffset = Math.max(-50, Math.min(50, lateralOffset));

                    if (Math.abs(lateralOffset) < 4.5) {
                        lateralOffset = 4.5 * Math.signum(lateralOffset);
                    }

                    double targetY = current.getY() + lateralOffset;
                    if (targetY < 0) targetY = 0;

                    Pose target = new Pose(
                            -1.5,
                            targetY,
                            current.getHeading()
                    );

                    PathChain correction = PedroComponent.follower().pathBuilder()
                            .addPath(new BezierLine(current, target))
                            .setConstantHeadingInterpolation(Math.toRadians(180))
                            .setConstraints(new PathConstraints(0.3, 100))
                            .setTValueConstraint(0.3)
                            .addParametricCallback(0.01, Intake.INSTANCE.locked)
                            .build();

                    PedroComponent.follower().followPath(correction, true);
                    startedAlignPath = true;
                })
                .setIsDone(() -> abortToLeave || (!PedroComponent.follower().isBusy() && startedAlignPath));
    }

    public Command backScore() {
        return new LambdaCommand()
                .setStart(() -> {
                    if (abortToLeave) return;
                    LimelightSubsystem.INSTANCE.artifact.invoke();

                    Pose current = PedroComponent.follower().getPose();

                    PathChain correction = PedroComponent.follower().pathBuilder()
                            .addPath(new BezierLine(current, scorePose))
                            .setConstraints(new PathConstraints(0.9, 100))
                            .setConstantHeadingInterpolation(Math.toRadians(180))
                            .build();

                    PedroComponent.follower().followPath(correction, true);
                })
                .setIsDone(() -> abortToLeave || !PedroComponent.follower().isBusy());
    }

    public Command leaveCommand() {
        return new LambdaCommand()
                .setStart(() -> {
                    LimelightSubsystem.INSTANCE.artifact.invoke();

                    Pose current = PedroComponent.follower().getPose();

                    PathChain correction = PedroComponent.follower().pathBuilder()
                            .addPath(new BezierLine(current, leavePose))
                            .setConstantHeadingInterpolation(Math.toRadians(0))
                            .build();

                    PedroComponent.follower().followPath(correction, true);
                })
                .setIsDone(() -> !PedroComponent.follower().isBusy());
    }

    public Command autonomousRoutine() {
        return new SequentialGroup(
                new InstantCommand(() -> {
                    LimelightSubsystem.INSTANCE.artifact.invoke(); Shooter.INSTANCE.switchVelocity(1220); Shooter.INSTANCE.switchHood(0.86); pos = 4450;
                }),
                new FollowPath(pathOne, true).and(Intake.INSTANCE.unlocked),
                new Delay(0.26), Intake.INSTANCE.coletAutoOn(), new Delay(0.29),
                new FollowPath(pathTwo, true), new FollowPath(pathThree, true),
                new Delay(0.001), Intake.INSTANCE.unlocked, new Delay(0.29),
                new FollowPath(pathFour, true), new FollowPath(pathFive, true), new FollowPath(pathSix, true),
                backScore(), new Delay(0.002), Intake.INSTANCE.unlocked, new Delay(0.29),
                alignToBallsSingleCorrection(),
                new Delay(0.1), backScore(), new Delay(0.002), Intake.INSTANCE.unlocked, new Delay(0.29),
                alignToBallsSingleCorrection(),
                new Delay(0.1), backScore(), new Delay(0.002), Intake.INSTANCE.unlocked, new Delay(0.29),
                alignToBallsSingleCorrection(),
                new Delay(0.1), backScore(), new Delay(0.002), Intake.INSTANCE.unlocked, new Delay(0.29),
                alignToBallsSingleCorrection(),
                new Delay(0.1), backScore(), new Delay(0.002), Intake.INSTANCE.unlocked, new Delay(0.29),
                alignToBallsSingleCorrection(),
                new Delay(0.1), backScore(), new Delay(0.002), Intake.INSTANCE.unlocked, new Delay(0.29),
                new FollowPath(pathLeave, true)



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
        autoTimer.reset();
        abortToLeave = false;
        forcedLeave = false;
        autonomousRoutine().schedule();
    }

    @Override
    public void onUpdate() {
        double timeLeft = 30.0 - autoTimer.seconds();

        if (!forcedLeave && timeLeft <= 1.0) {
            forcedLeave = true;
            abortToLeave = true;
            leavePathStarted = false;

            PedroComponent.follower().breakFollowing();
        }

        if (abortToLeave && !leavePathStarted && !PedroComponent.follower().isBusy()) {
            Pose current = PedroComponent.follower().getPose();

            PathChain leavePath = PedroComponent.follower().pathBuilder()
                    .addPath(new BezierLine(current, leavePose))
                    .setLinearHeadingInterpolation(current.getHeading(), leavePose.getHeading())
                    .build();

            PedroComponent.follower().followPath(leavePath, true);
            leavePathStarted = true;
        }

        panelsTelemetry.debug("X", PedroComponent.follower().getPose().getX());
        panelsTelemetry.debug("Y", PedroComponent.follower().getPose().getY());
        panelsTelemetry.debug("Heading", Math.toDegrees(PedroComponent.follower().getPose().getHeading()));
        panelsTelemetry.debug("Target Shooter", Shooter.INSTANCE.velocityAuto);
        panelsTelemetry.debug("Current Shooter", Shooter.INSTANCE.currentVelocity);
        panelsTelemetry.debug("tx", LimelightSubsystem.INSTANCE.tx);
        panelsTelemetry.debug("Current Ticks", testTurret.INSTANCE.currentTicks);

        if (abortToLeave) {
            panelsTelemetry.debug("AUTO", "FORCING LEAVE");
        }
        testTurret.INSTANCE.turretToPosition(pos);

        panelsTelemetry.update(telemetry);

        Shooter.INSTANCE.shooterAutoOn().invoke();
    }

    @Override
    public void onStop() {
        autoEndPoseBlue = PedroComponent.follower().getPose();
    }
}