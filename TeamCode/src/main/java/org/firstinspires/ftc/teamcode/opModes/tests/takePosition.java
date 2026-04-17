package org.firstinspires.ftc.teamcode.opModes.tests;

import static org.firstinspires.ftc.teamcode.Subsystems.Led.PrismState.GREEN_SNAKE;
import static org.firstinspires.ftc.teamcode.Subsystems.Led.PrismState.GREEN_SOLID;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.teamcode.Mecanismos.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.EndGame;
import org.firstinspires.ftc.teamcode.Subsystems.Led;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name="takePosition", group = "OpModes Tests")
public class takePosition extends NextFTCOpMode {

    public takePosition() {
        addComponents(
                new SubsystemComponent(LimelightSubsystem.INSTANCE, Led.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    int count = 0;

    private boolean endGameToggle = false;
    private boolean endGameLastButtonState = false;

    private boolean alignToggle = false;
    private boolean alignLastButtonState = false;


    private final Pose startPose = new Pose(0, 0, Math.toRadians(0));

    MecanumDrive mecanumDrive = new MecanumDrive();

    public static TelemetryManager telemetryMa;

    private final PolygonZone closeLaunchZone = new PolygonZone(new Point(134, 134), new Point(68, 68), new Point(0, 134));
    private final PolygonZone farLaunchZone = new PolygonZone(new Point(48.5, 0), new Point(63, 14), new Point(77, 0));
    private final PolygonZone blueBase = new PolygonZone(new Point(95.5, 23.5), 20, 20);
    private final PolygonZone redBase = new PolygonZone(new Point(28.5, 23.5), 20, 20);

    private final PolygonZone robotZone = new PolygonZone(16, 14);

    @Override
    public void onInit() {
        telemetryMa = PanelsTelemetry.INSTANCE.getTelemetry();
        PedroComponent.follower().setStartingPose(startPose);
        PedroComponent.follower().updatePose();
        LimelightSubsystem.INSTANCE.switchPipeline(1);
        LimelightSubsystem.INSTANCE.setTeleOp(true);
        telemetryMa.addData("Status:", "inicializado");
        telemetryMa.update(telemetry);
    }

    @Override
    public void onStartButtonPressed() {
        mecanumDrive.start();
    }

    @Override
    public void onUpdate() {
        PedroComponent.follower().update();
        double x = PedroComponent.follower().getPose().getX();
        double y = PedroComponent.follower().getPose().getY();
        double heading = PedroComponent.follower().getPose().getHeading();
        double vx = PedroComponent.follower().getVelocity().getXComponent();
        double vy = PedroComponent.follower().getVelocity().getYComponent();

        robotZone.setPosition(PedroComponent.follower().getPose().getX(), PedroComponent.follower().getPose().getY());
        robotZone.setRotation(PedroComponent.follower().getPose().getHeading());

        if (robotZone.isInside(closeLaunchZone) || robotZone.isInside(farLaunchZone)) {
            Led.INSTANCE.setPrismState(GREEN_SNAKE);
        } else {
            Led.INSTANCE.setPrismState(GREEN_SOLID);
        }

        //Toggle EndGame
        boolean endGameCurrentButtonState = gamepad1.dpad_down;
        if (endGameCurrentButtonState && !endGameLastButtonState) {
            endGameToggle = !endGameToggle;
        }
        endGameLastButtonState = endGameCurrentButtonState;

        if (gamepad1.a) {
            PedroComponent.follower().setPose(new Pose(LimelightSubsystem.INSTANCE.pedroY, LimelightSubsystem.INSTANCE.pedroX, LimelightSubsystem.INSTANCE.headingRadians));
        }

        if (count == 0) {
            LimelightSubsystem.INSTANCE.relocalization.invoke();
        }

        telemetryMa.addData("Odometria", "------------------");
        telemetryMa.addData("X", x);
        telemetryMa.addData("Y", y);
        telemetryMa.addData("vX", vx);
        telemetryMa.addData("vY", vy);
        telemetryMa.addData("Heading", Math.toDegrees(heading));
        telemetryMa.update(telemetry);
    }

    @Override
    public void onStop() {
    }
}
