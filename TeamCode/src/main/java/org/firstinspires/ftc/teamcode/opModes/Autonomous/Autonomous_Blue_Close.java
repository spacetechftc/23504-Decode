package org.firstinspires.ftc.teamcode.opModes.Autonomous;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnTo;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name="12 Ball - Blue Close", preselectTeleOp = "TeleOp - Blue")
@Configurable
public class Autonomous_Blue_Close extends NextFTCOpMode {
    public Autonomous_Blue_Close() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Turret.INSTANCE, LimelightSubsystem.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }
    private TelemetryManager panelsTelemetry;
    public static Pose autoEndPose;

    // Definição das coordenadas
    private final Pose startPose = new Pose(39, 120, Math.toRadians(143));
    private final Pose scorePose = new Pose(70, 78, Math.toRadians(180));

    private final Pose initGate = new Pose(31, 69, Math.toRadians(90));
    private final Pose openGate = new Pose(29, 69, Math.toRadians(90));

    private final Pose takeBalls_1 = new Pose(34, 78, Math.toRadians(180));
    private final Pose takeBalls_2 = new Pose(28, 54, Math.toRadians(180));
    private final Pose takeBalls_3 = new Pose(28, 29, Math.toRadians(180));

    private final Pose intakeBalls_2 = new Pose(59, 54, Math.toRadians(180));
    private final Pose intakeBalls_3 = new Pose(59, 29, Math.toRadians(180));

    private final Pose goBack = new Pose(32, 52, Math.toRadians(180));
    private final Pose endPose = new Pose(40, 69, Math.toRadians(270));

    private Follower follower;
    private Path pathOne, pathTwo, pathThree, pathFour, pathFive, pathBack, pathSix, pathSeven, pathEight, pathNine, pathTen, pathInitGate, pathOpenGate;

    public void buildPaths() {
        pathOne = new Path(new BezierLine(startPose, scorePose));
        pathOne.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        pathTwo = new Path(new BezierLine(scorePose, takeBalls_1));
        pathInitGate = new Path(new BezierLine(takeBalls_1, initGate));
        pathInitGate.setLinearHeadingInterpolation(takeBalls_1.getHeading(), initGate.getHeading());
        pathOpenGate = new Path(new BezierLine(initGate, openGate));
        pathOpenGate.setLinearHeadingInterpolation(initGate.getHeading(), openGate.getHeading());
        pathTwo.setLinearHeadingInterpolation(scorePose.getHeading(), takeBalls_1.getHeading());
        pathThree = new Path(new BezierLine(openGate, scorePose));
        pathThree.setLinearHeadingInterpolation(openGate.getHeading(), scorePose.getHeading());
        pathFour = new Path(new BezierLine(scorePose, intakeBalls_2));
        pathFour.setLinearHeadingInterpolation(scorePose.getHeading(), intakeBalls_2.getHeading());
        pathFive = new Path(new BezierLine(intakeBalls_2, takeBalls_2));
        pathFive.setLinearHeadingInterpolation(intakeBalls_2.getHeading(), takeBalls_2.getHeading());
        pathBack = new Path(new BezierLine(takeBalls_2, goBack));
        pathBack.setLinearHeadingInterpolation(takeBalls_1.getHeading(), goBack.getHeading());
        pathSix = new Path(new BezierLine(goBack, scorePose));
        pathSix.setLinearHeadingInterpolation(goBack.getHeading(), scorePose.getHeading());
        pathSeven = new Path(new BezierLine(scorePose, intakeBalls_3));
        pathSeven.setLinearHeadingInterpolation(scorePose.getHeading(), intakeBalls_3.getHeading());
        pathEight = new Path(new BezierLine(intakeBalls_3, takeBalls_3));
        pathEight.setLinearHeadingInterpolation(intakeBalls_3.getHeading(), takeBalls_3.getHeading());
        pathNine = new Path(new BezierLine(takeBalls_3, scorePose));
        pathNine.setLinearHeadingInterpolation(takeBalls_3.getHeading(), scorePose.getHeading());
        pathTen = new Path(new BezierLine(scorePose, endPose));
        pathTen.setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading());

    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                Shooter.INSTANCE.shooterAutoOn(),
                new FollowPath(pathOne),  // Indo pro scorePose
                new Delay(0.2),
                Intake.INSTANCE.coletAutoOn(),
                new Delay(1.1),
                Shooter.INSTANCE.shooterAutoNegative(),
                new FollowPath(pathTwo),
                new Delay(0.2),
                Intake.INSTANCE.stopAuto(),
                new FollowPath(pathInitGate),
                new FollowPath(pathOpenGate),
                new Delay(0.85),
                Shooter.INSTANCE.shooterAutoOn(),
                new FollowPath(pathThree),
                new Delay(0.3),
                Intake.INSTANCE.coletAutoOn(),
                new Delay(1.1),
                Intake.INSTANCE.stopAuto(),
                new FollowPath(pathFour),
                Intake.INSTANCE.coletAutoOn(),
                Shooter.INSTANCE.shooterAutoNegative(),
                new FollowPath(pathFive),
                new Delay(0.2),
                new FollowPath(pathBack),
                new Delay(0.1),
                Intake.INSTANCE.stopAuto(),
                Shooter.INSTANCE.shooterAutoOn(),
                new FollowPath(pathSix),
                new Delay(0.4),
                Intake.INSTANCE.coletAutoOn(),
                new Delay(1.1),
                Intake.INSTANCE.stopAuto(),
                new FollowPath(pathSeven),
                Shooter.INSTANCE.shooterAutoNegative(),
                Intake.INSTANCE.coletAutoOn(),
                new FollowPath(pathEight),
                new Delay(0.2),
                Intake.INSTANCE.stopAuto(),
                Shooter.INSTANCE.shooterAutoOn(),
                new FollowPath(pathNine),
                new Delay(0.4),
                Intake.INSTANCE.coletAutoOn(),
                new Delay(1.1),
                new FollowPath(pathTen),
                Intake.INSTANCE.stopAuto()

        );
    }

    @Override
    public void onInit() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        new InstantCommand(() -> {
            Turret.INSTANCE.resetEncoder.invoke();
        });
        new InstantCommand(() -> {
            LimelightSubsystem.INSTANCE.switchIndexBlue.invoke();
        });
        follower.setStartingPose(startPose);
        buildPaths();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void onWaitForStart() {
        new InstantCommand(() -> {
            Turret.INSTANCE.resetEncoder.invoke();
        });
        LimelightSubsystem.INSTANCE.switchIndexBlue.invoke();
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
        Turret.INSTANCE.alignTurret(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), Turret.INSTANCE.currentTicks, true);
        panelsTelemetry.update(telemetry);
        telemetry.update();
        follower.update();
    }

    @Override
    public void onStop() {
        autoEndPose = follower.getPose();
    }
}

