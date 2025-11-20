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

@Autonomous(name="12 Ball Red Side Classifier Auto")
@Configurable
public class Autonomous_Red extends NextFTCOpMode {
    public Autonomous_Red() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Turret.INSTANCE, LimelightSubsystem.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }
    private TelemetryManager panelsTelemetry;

    // Definição das coordenadas
    private final Pose startPose = new Pose(131.215, 118.654, Math.toRadians(36));
    private final Pose scorePose = new Pose(105.888, 85.682, Math.toRadians(45));
    private final Pose initGate = new Pose(136, 63, Math.toRadians(96));
    private final Pose openGate = new Pose(142.5, 63, Math.toRadians(96));
    private final Pose intakeBalls_1 = new Pose(110.888, 76.682, Math.toRadians(0));
    private final Pose intakeBalls_2 = new Pose(110.888,  53, Math.toRadians(0));
    private final Pose intakeBalls_3 = new Pose(110.888, 30, Math.toRadians(0));
    private final Pose takeBalls_1 = new Pose(139, 74.682, Math.toRadians(0));
    private final Pose takeBalls_2 = new Pose(145, 51, Math.toRadians(0));
    private final Pose goBack = new Pose(141, 51, Math.toRadians(0));
    private final Pose takeBalls_3 = new Pose(145, 30, Math.toRadians(0));
    private final Pose endPose = new Pose(130.206, 60.121, Math.toRadians(270));

    private Follower follower;
    private Path pathOne, pathOneTwo, pathTwo, pathThree, pathFour, pathFive, pathBack, pathSix, pathSeven, pathEight, pathNine, pathTen, pathInitGate ,pathOpenGate;

    public void buildPaths() {
        pathOne = new Path(new BezierLine(startPose, scorePose));
        pathOne.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        pathOneTwo = new Path(new BezierLine(scorePose, intakeBalls_1));
        pathOneTwo.setLinearHeadingInterpolation(scorePose.getHeading(), intakeBalls_1.getHeading());
        pathTwo = new Path(new BezierLine(intakeBalls_1, takeBalls_1));
        pathTwo.setConstantHeadingInterpolation(0);
        pathThree = new Path(new BezierLine(openGate, scorePose));
        pathThree.setLinearHeadingInterpolation(openGate.getHeading(), scorePose.getHeading());
        pathFour = new Path(new BezierLine(scorePose, intakeBalls_2));
        pathFour.setConstantHeadingInterpolation(0);
        pathFive = new Path(new BezierLine(intakeBalls_2, takeBalls_2));
        pathFive.setConstantHeadingInterpolation(0);
        pathSix = new Path(new BezierLine(takeBalls_2, scorePose));
        pathSix.setLinearHeadingInterpolation(takeBalls_2.getHeading(), scorePose.getHeading());
        pathSeven = new Path(new BezierLine(scorePose, intakeBalls_3));
        pathSeven.setLinearHeadingInterpolation(scorePose.getHeading(), intakeBalls_3.getHeading());
        pathEight = new Path(new BezierLine(intakeBalls_3, takeBalls_3));
        pathEight.setConstantHeadingInterpolation(0);
        pathNine = new Path(new BezierLine(takeBalls_3, scorePose));
        pathNine.setLinearHeadingInterpolation(takeBalls_3.getHeading(), scorePose.getHeading());
        pathTen = new Path(new BezierLine(scorePose, endPose));
        pathTen.setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading());
        pathInitGate = new Path(new BezierLine(takeBalls_1, initGate));
        pathInitGate.setLinearHeadingInterpolation(takeBalls_1.getHeading(), initGate.getHeading());
        pathOpenGate = new Path(new BezierLine(initGate, openGate));
        pathOpenGate.setLinearHeadingInterpolation(initGate.getHeading(), openGate.getHeading());
        pathBack = new Path(new BezierLine(takeBalls_2, goBack));
        pathBack.setConstantHeadingInterpolation(0);
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                Shooter.INSTANCE.shooterAutoOn(),
                new FollowPath(pathOne), // Indo pro scorePose
                new Delay(0.3), // Limelight se alinhando
                Intake.INSTANCE.coletAutoOn(),
                new Delay(1.3),
                Intake.INSTANCE.stopAuto(), // Lançou as primeiras bolas
                new FollowPath(pathOneTwo),
                new ParallelGroup(new FollowPath(pathTwo), Intake.INSTANCE.coletAutoOn(), Shooter.INSTANCE.shooterAutoNegative()), // Pegou a primeira fileira
                new Delay(0.4),
                new ParallelGroup(new FollowPath(pathInitGate), Intake.INSTANCE.stopAuto()), // Indo abrir o gate
                new FollowPath(pathInitGate, false, 0.02),
                new FollowPath(pathOpenGate), // Abriu o gate
                new Delay(1.45), // Espera o gate
                Shooter.INSTANCE.shooterAutoOn(),
                new FollowPath(pathThree), // Indo pro scorePose
                new Delay(0.3), // Limelight se alinhando
                Intake.INSTANCE.coletAutoOn(),
                new Delay(1), // Lançou a primeira fileira
                Intake.INSTANCE.stopAuto(),
                new FollowPath(pathFour),
                new ParallelGroup(new FollowPath(pathFive), Intake.INSTANCE.coletAutoOn(), Shooter.INSTANCE.shooterAutoNegative()), // Pegou a segunda fileira
                new Delay(0.3),
                new FollowPath(pathBack),
                new Delay(0.1),
                Intake.INSTANCE.stopAuto(),
                Shooter.INSTANCE.shooterAutoOn(),
                new FollowPath(pathSix), // Indo pro scorePose
                new Delay(0.3), // Limelight se alinhando
                Intake.INSTANCE.coletAutoOn(),
                new Delay(1.3), // Lançou a segunda fileira
                Intake.INSTANCE.stopAuto(),
                new FollowPath(pathSeven),
                new Delay(0.15), // Aguentando o supapo
                new ParallelGroup(new FollowPath(pathEight), Intake.INSTANCE.coletAutoOn(), Shooter.INSTANCE.shooterAutoNegative()), // Pegou a terceira fileira
                new Delay(0.4),
                Intake.INSTANCE.stopAuto(),
                Shooter.INSTANCE.shooterAutoOn(),
                new FollowPath(pathNine), // Indo pro scorePose
                new Delay(0.4), // Limelight se alinhando
                Intake.INSTANCE.coletAutoOn(),
                new Delay(1.3),
                Intake.INSTANCE.stopAuto(),
                new FollowPath(pathTen)

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
        panelsTelemetry.update(telemetry);
        telemetry.update();
        follower.update();
    }

}

