package org.firstinspires.ftc.teamcode.opModes.Autonomous;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
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
public class Autonomous_Red extends NextFTCOpMode {
    public Autonomous_Red() {
        addComponents(
                new SubsystemComponent(),
                new PedroComponent(Constants::createFollower),
                BulkReadComponent.INSTANCE
        );
    }

    // Definição das coordenadas
    private final Pose startPose = new Pose(181.654, 131.215, Math.toRadians(-144)).mirror();

    private final Pose scorePose = new Pose(85.682, 83.888, Math.toRadians(360)).mirror();
    private final Pose intakeBalls_2 = new Pose(85.682,  59.664).mirror();
    private final Pose intakeBalls_3 = new Pose(85.862, 35.664).mirror();

    private final Pose takeBalls_1 = new Pose(125.607, 83.664, Math.toRadians(0)).mirror();
    private final Pose takeBalls_2 = new Pose(125.607, 59.664).mirror();
    private final Pose takeBalls_3 = new Pose(125.607, 35.664).mirror();

    private final Pose endPose = new Pose(121.121, 70.206, Math.toRadians(180)).mirror();

    private Follower follower;
    private PathChain pathOne, pathTwo, pathThree, pathFour, pathFive, pathSix, pathSeven, pathEight, pathNine, pathTen;


    public void buildPaths() {
        pathOne = follower.pathBuilder()
                .addPath(new BezierLine(startPose, scorePose))
                .setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading())
                .build();
        pathTwo = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, takeBalls_1))
                .setTangentHeadingInterpolation()
                .build();
        pathThree = follower.pathBuilder()
                .addPath(new BezierLine(takeBalls_1, scorePose))
                .setConstantHeadingInterpolation(takeBalls_1.getHeading())
                .build();
        pathFour = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intakeBalls_2))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();
        pathFive = follower.pathBuilder()
                .addPath(new BezierLine(intakeBalls_2, takeBalls_2))
                .setTangentHeadingInterpolation()
                .build();
        pathSix = follower.pathBuilder()
                .addPath(new BezierLine(takeBalls_2, scorePose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();
        pathSeven = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, intakeBalls_3))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();
        pathEight = follower.pathBuilder()
                .addPath(new BezierLine(intakeBalls_3, takeBalls_3))
                .setTangentHeadingInterpolation()
                .build();
        pathNine = follower.pathBuilder()
                .addPath(new BezierLine(takeBalls_3, scorePose))
                .setConstantHeadingInterpolation(scorePose.getHeading())
                .build();
        pathTen = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, endPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), endPose.getHeading())
                .build();
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
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startPose);
        buildPaths();
    }

    @Override
    public void onStartButtonPressed() {
        autonomousRoutine().schedule();
    }

    @Override
    public void onUpdate() {
        follower.update();
    }
}

