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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@Autonomous(name="12 Ball Red Side Classifier Auto")
@Configurable
public class Autonomous_Red extends NextFTCOpMode {
    public Autonomous_Red() {
        addComponents(
                new SubsystemComponent(),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }
    private TelemetryManager panelsTelemetry;
    // X E Y 1
    // Y E X 2
    // Definição das coordenadas
    private final Pose startPose = new Pose(131.215, 118.654, Math.toRadians(216));
    private final Pose scorePose = new Pose(96.888, 76.682, Math.toRadians(0));
    private final Pose intakeBalls_2 = new Pose(96.888,  53, Math.toRadians(0));
    private final Pose intakeBalls_3 = new Pose(96.888, 30, Math.toRadians(0));
   private final Pose takeBalls_1 = new Pose(125, 76.682, Math.toRadians(0));
   private final Pose takeBalls_2 = new Pose(125, 53, Math.toRadians(0));
   private final Pose takeBalls_3 = new Pose(125, 30, Math.toRadians(0));
    private final Pose endPose = new Pose(130.206, 60.121, Math.toRadians(270));

    private Follower follower;
  //  private PathChain pathOne, pathTwo, pathThree, pathFour, pathFive, pathSix, pathSeven, pathEight, pathNine, pathTen;
    private Path pathOne, pathTwo, pathThree, pathFour, pathFive, pathSix, pathSeven, pathEight, pathNine, pathTen;


    public void buildPaths() {
        pathOne = new Path(new BezierLine(startPose, scorePose));
        pathOne.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());
        pathTwo = new Path(new BezierLine(scorePose, takeBalls_1));
        pathTwo.setTangentHeadingInterpolation();
        pathThree = new Path(new BezierLine(takeBalls_1, scorePose));
        pathThree.setConstantHeadingInterpolation(0);
        pathFour = new Path(new BezierLine(scorePose, intakeBalls_2));
        pathFour.setConstantHeadingInterpolation(0);
        pathFive = new Path(new BezierLine(intakeBalls_2, takeBalls_2));
        pathFive.setConstantHeadingInterpolation(0);
        pathSix = new Path(new BezierLine(takeBalls_2, scorePose));
        pathSix.setConstantHeadingInterpolation(0);
        pathSeven = new Path(new BezierLine(scorePose, intakeBalls_3));
        pathSeven.setConstantHeadingInterpolation(0);
        pathEight = new Path(new BezierLine(intakeBalls_3, takeBalls_3));
        pathEight.setConstantHeadingInterpolation(0);
        pathNine = new Path(new BezierLine(takeBalls_3, scorePose));
        pathNine.setConstantHeadingInterpolation(0);
        pathTen = new Path(new BezierLine(scorePose, endPose));
        pathTen.setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading());
    }

    private Command autonomousRoutine() {
        return new SequentialGroup(
                new FollowPath(pathOne),
                new FollowPath(pathTwo),
                new FollowPath(pathThree),
                new FollowPath(pathFour),
                new FollowPath(pathFive),
                new FollowPath(pathSix),
                new FollowPath(pathSeven),
                new FollowPath(pathEight),
                new FollowPath(pathNine),
                new FollowPath(pathTen)
        );
    }

    @Override
    public void onInit() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
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
        follower.update();
    }
}

