package org.firstinspires.ftc.teamcode.opModes.Autonomous;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static org.firstinspires.ftc.teamcode.opModes.Autonomous.Autonomous_Red_Close.autoEndPose;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name="Auto Red - Far")
public class Autonomous_Red_Far extends NextFTCOpMode {
    public Autonomous_Red_Far() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Turret.INSTANCE, LimelightSubsystem.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }
    private TelemetryManager panelsTelemetry;

    // Definição das coordenadas
    private final Pose startPose = new Pose(101, -4.707, Math.toRadians(92));
    private final Pose endPose = new Pose(127.206, 0, Math.toRadians(90));

    private Follower follower;
    private Path pathOne;

    public void buildPaths() {
        pathOne = new Path(new BezierLine(startPose, endPose));
        pathOne.setConstantHeadingInterpolation(Math.toRadians(90));
    }

    private Command shootBalls() {
        return new SequentialGroup(
                Shooter.INSTANCE.shooterAutoOnFar(),
                new Delay(3),
                Intake.INSTANCE.coletAutoOn(),
                new Delay(0.2),// Lançou a primeira
                Intake.INSTANCE.stopAuto(),
                new Delay(0.8),
                Intake.INSTANCE.coletAutoOn(),
                new Delay(0.25), // Lançou a segunda
                Intake.INSTANCE.stopAuto(),
                new Delay(1),
                Intake.INSTANCE.coletAutoOn(),
                new Delay(0.3), // Lançou a terceira
                Intake.INSTANCE.stopAuto(),
                new Delay(0.6),
                Intake.INSTANCE.coletAutoOn(),
                new Delay(0.3), // Caso não dê pra lançar a terceira
                Intake.INSTANCE.stopAuto()
        );
    }
    private Command autonomousRoutine() {
        return new SequentialGroup(
            shootBalls(),
                new FollowPath(pathOne)
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
            LimelightSubsystem.INSTANCE.switchIndexRed.invoke();
        });
        follower.setStartingPose(startPose);
        buildPaths();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void onWaitForStart() {
        new InstantCommand(() -> {
            LimelightSubsystem.INSTANCE.switchIndexRed.invoke();
        });
        new InstantCommand(() -> {
            Turret.INSTANCE.resetEncoder.invoke();
        });
        LimelightSubsystem.INSTANCE.switchIndexRed.invoke();
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
        Turret.INSTANCE.alignTurret(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), Turret.INSTANCE.currentTicks, false);
        panelsTelemetry.update(telemetry);
        telemetry.update();
        follower.update();
    }

    @Override
    public void onStop() {
        autoEndPose = follower.getPose();
    }

}



