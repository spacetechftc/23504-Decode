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
import org.firstinspires.ftc.teamcode.Subsystems.WebcamSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.core.units.Angle;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.extensions.pedro.TurnTo;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name="Testing Correcton", preselectTeleOp = "TeleOp - Red")
@Configurable
public class testAuto extends NextFTCOpMode {
    public testAuto() {
        addComponents(
                new SubsystemComponent(Intake.INSTANCE, Shooter.INSTANCE, Turret.INSTANCE, LimelightSubsystem.INSTANCE, WebcamSubsystem.INSTANCE),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }
    private TelemetryManager panelsTelemetry;
    public static Pose autoEndPose;

    // Definição das coordenadas
    private final Pose startPose = new Pose(131.215, 118.654, Math.toRadians(36));
    //private final Pose scorePose = new Pose(105.888, 85.682, Math.toRadians(45));

    private final Pose scorePose = new Pose(108.888, 74.682, Math.toRadians(0));

    private final Pose initGate = new Pose(136, 62.7, Math.toRadians(96));
    private final Pose openGate = new Pose(137.2, 62.7, Math.toRadians(96));
    private final Pose intakeBalls_2 = new Pose(110.888,  53, Math.toRadians(0));
    private final Pose intakeBalls_3 = new Pose(110.888, 30, Math.toRadians(0));
    private final Pose takeBalls_1 = new Pose(139, 74.682, Math.toRadians(0));
    private final Pose takeBalls_2 = new Pose(144, 51, Math.toRadians(0));
    private final Pose goBack = new Pose(138, 51, Math.toRadians(0));
    private final Pose takeBalls_3 = new Pose(145, 30, Math.toRadians(0));
    private final Pose endPose = new Pose(130.206, 60.121, Math.toRadians(270));

    private Pose correctionTakeBall1 = new Pose(108.888, 72.682 + WebcamSubsystem.INSTANCE.getCorrection(), Math.toRadians(0));

    private Follower follower;
    private Path pathOne, pathTwo, pathThree, pathFour, pathFive, pathBack, pathSix, pathSeven, pathEight, pathNine, pathTen, pathInitGate ,pathOpenGate;

    public void buildPaths() {
        pathOne = new Path(new BezierLine(startPose, scorePose));
        pathOne.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        pathTwo = new Path(new BezierLine(scorePose, correctionTakeBall1));
        pathTwo.setLinearHeadingInterpolation(scorePose.getHeading(), correctionTakeBall1.getHeading());
        /*
        pathTwo = new Path(new BezierLine(scorePose, correctionTakeBall1));
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

         */

    }


    private Command autonomousRoutine() {
        return new SequentialGroup(
                new FollowPath(pathOne), // Indo pro scorePose
                new Delay(2),
                new FollowPath(pathTwo)


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
        panelsTelemetry.debug("Pixel X", WebcamSubsystem.INSTANCE.x);
        panelsTelemetry.debug("Delta Y", WebcamSubsystem.INSTANCE.getCorrection());
        //Turret.INSTANCE.alignTurret(follower.getPose().getX(), follower.getPose().getY(), follower.getPose().getHeading(), Turret.INSTANCE.currentTicks, false);
        panelsTelemetry.update(telemetry);
        telemetry.update();
        follower.update();


    }

    @Override
    public void onStop() {
        autoEndPose = follower.getPose();
    }
}

