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

@Autonomous(name="12 Ball - Blue Close")
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
    private final Pose startPose = new Pose(37, 120, Math.toRadians(143));
    private final Pose scorePose = new Pose(59, 75, Math.toRadians(178));

    private final Pose initGate = new Pose(31, 69, Math.toRadians(90));
    private final Pose openGate = new Pose(20, 69, Math.toRadians(90));

    private final Pose intakeBalls_2 = new Pose(54, 48, Math.toRadians(178));
    private final Pose takeBalls_1 = new Pose(34, 75, Math.toRadians(178));

    private Follower follower;
    private Path pathOne, pathTwo, pathThree, pathFour, pathFive, pathBack, pathSix, pathSeven, pathEight, pathNine, pathTen, pathInitGate ,pathOpenGate;

    public void buildPaths() {
        pathOne = new Path(new BezierLine(startPose, scorePose));
        pathOne.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        pathTwo = new Path(new BezierLine(scorePose, takeBalls_1));
        pathTwo.setLinearHeadingInterpolation(scorePose.getHeading(), takeBalls_1.getHeading());
        pathThree = new Path(new BezierLine(takeBalls_1, scorePose));
        pathThree.setLinearHeadingInterpolation(takeBalls_1.getHeading(), scorePose.getHeading());
        pathFour = new Path(new BezierLine(scorePose, intakeBalls_2));
        pathFour.setLinearHeadingInterpolation(scorePose.getHeading(), intakeBalls_2.getHeading());


    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                Shooter.INSTANCE.shooterAutoOn(),
                new FollowPath(pathOne),  // Indo pro scorePose
                Intake.INSTANCE.coletAutoOn(),
                new Delay(1.1),
                Shooter.INSTANCE.shooterAutoNegative(),
                new FollowPath(pathTwo),
                new Delay(0.2),
                Intake.INSTANCE.stopAuto(),
                Shooter.INSTANCE.shooterAutoOn(),
                new FollowPath(pathThree),
                Intake.INSTANCE.coletAutoOn(),
                new Delay(1.1),
                Shooter.INSTANCE.shooterAutoNegative(),
                Intake.INSTANCE.stopAuto(),
                new FollowPath(pathFour)


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
            LimelightSubsystem.INSTANCE.switchIndexBlue.invoke();
        });
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
        //Turret.INSTANCE.alignTurret(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), Turret.INSTANCE.currentTicks, true);
        panelsTelemetry.update(telemetry);
        telemetry.update();
        follower.update();
    }

    @Override
    public void onStop() {
        autoEndPose = follower.getPose();
    }
}

