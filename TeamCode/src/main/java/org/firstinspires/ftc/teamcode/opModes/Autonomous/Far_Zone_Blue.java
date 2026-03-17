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

import static org.firstinspires.ftc.teamcode.opModes.Autonomous.Eighteen_Balls_Close_Blue.autoEndPoseBlue;


@Autonomous(name="Far_Zone_Blue", group = "Auto Far Blue", preselectTeleOp = "TeleOp_Blue")
@Configurable
public class Far_Zone_Blue extends NextFTCOpMode {

    public Far_Zone_Blue() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, testTurret.INSTANCE, Led.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    private TelemetryManager panelsTelemetry;

    private final Pose startPose = new Pose(51.4, 2.8, Math.toRadians(270));
    private final Pose scorePose = new Pose(52, 8, Math.toRadians(180));

    private final Pose takeBalls_Down = new Pose(7, 29, Math.toRadians(180));
    private final Pose takeBalls_Player_One = new Pose(7, 9, Math.toRadians(180));
    private final Pose takeBalls_Player_Back = new Pose(23, 5, Math.toRadians(180));
    private final Pose takeBalls_Player_Two = new Pose(7, -2, Math.toRadians(180));

    private final Pose leave  = new Pose(30, 10, Math.toRadians(90));

    private Path pathOne, pathTwo, pathScore, pathPlayerOne, pathPlayerBack, pathPlayerTwo, pathLeave;
    private PathChain pathDown;

    private Command shootBalls() {
        return new SequentialGroup(
          Intake.INSTANCE.coletAutoOn(),
          new Delay(0.02),
          Intake.INSTANCE.stopAuto(),
          new Delay(0.45),
          Intake.INSTANCE.coletAutoOn(),
          new Delay(0.6)
        );
    }


    public void buildPaths(Follower follower) {
        pathOne = new Path(new BezierLine(startPose, scorePose));
        pathOne.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

        pathDown = PedroComponent.follower().pathBuilder()
                .addPath(
                        new BezierCurve(
                                scorePose,
                                new Pose(35,29),
                                takeBalls_Down
                        )
                )
                .setConstantHeadingInterpolation(Math.toRadians(180))
                .addParametricCallback(0.45, ()-> PedroComponent.follower().setMaxPowerScaling(0.65))
                .addParametricCallback(0.97, ()-> PedroComponent.follower().setMaxPowerScaling(1))
                .build();

        pathTwo = new Path(new BezierLine(takeBalls_Down, scorePose));
        pathTwo.setConstantHeadingInterpolation(Math.toRadians(180));

        pathPlayerOne = new Path(new BezierLine(scorePose, takeBalls_Player_One));
        pathPlayerOne.setConstantHeadingInterpolation(Math.toRadians(180));

        pathPlayerBack = new Path(new BezierLine(takeBalls_Player_One, takeBalls_Player_Back));
        pathPlayerBack.setConstantHeadingInterpolation(Math.toRadians(180));

        pathPlayerTwo = new Path(new BezierLine(takeBalls_Player_Back, takeBalls_Player_Two));
        pathPlayerTwo.setConstantHeadingInterpolation(Math.toRadians(180));


        pathScore = new Path(new BezierLine(takeBalls_Player_Two, scorePose));
        pathScore.setConstantHeadingInterpolation(Math.toRadians(180));

        pathLeave = new Path(new BezierLine(scorePose, leave));
        pathLeave.setLinearHeadingInterpolation(scorePose.getHeading(), leave.getHeading());
    }

    public Command autonomousRoutine() {
        return new SequentialGroup(
                new InstantCommand(() -> {
                    Shooter.INSTANCE.switchHood(0.9); Shooter.INSTANCE.switchVelocity(1550); Shooter.INSTANCE.shooterAutoOn();
                }),
                new FollowPath(pathOne, true),
                new Delay(1.5),
                shootBalls(),
                new FollowPath(pathDown).and(Intake.INSTANCE.locked),
                new Delay(0.08),
                Intake.INSTANCE.stopAuto(),
                new FollowPath(pathTwo, true, 0.8).and(Intake.INSTANCE.unlocked),
                new Delay(0.3),
                shootBalls(),
                new FollowPath(pathPlayerOne, true, 0.7).and(Intake.INSTANCE.locked),
                new Delay(0.15),
                new FollowPath(pathPlayerBack, true, 0.85),
                new Delay(0.01),
                new FollowPath(pathPlayerTwo,true, 0.7),
                new Delay(0.1),
                Intake.INSTANCE.stopAuto(),
                new FollowPath(pathScore, true, 0.75).and(Intake.INSTANCE.unlocked),
                new Delay(0.3),
                shootBalls(),
                new FollowPath(pathPlayerOne, true, 0.7).and(Intake.INSTANCE.locked),
                new Delay(0.15),
                new FollowPath(pathPlayerBack, true, 0.85),
                new Delay(0.01),
                new FollowPath(pathPlayerTwo,true, 0.7),
                new Delay(0.1),
                Intake.INSTANCE.stopAuto(),
                new FollowPath(pathScore, true, 0.75).and(Intake.INSTANCE.unlocked),
                new Delay(0.3),
                shootBalls(),
                new FollowPath(pathLeave),
                Intake.INSTANCE.stopAuto(),
                new InstantCommand(() -> Shooter.INSTANCE.stopAuto())

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



