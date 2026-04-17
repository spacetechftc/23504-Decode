package org.firstinspires.ftc.teamcode.opModes.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.skeletonarmy.marrow.TimerEx;
import com.skeletonarmy.marrow.zones.Point;
import com.skeletonarmy.marrow.zones.PolygonZone;

import org.firstinspires.ftc.teamcode.Mecanismos.MecanumDrive;
import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.core.components.SubsystemComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.ftc.components.BulkReadComponent;

@TeleOp(name="testLimelight", group = "OpModes Tests")
public class testLimelight extends NextFTCOpMode {

    public testLimelight() {
        addComponents(
                new SubsystemComponent(LimelightSubsystem.INSTANCE),
                BulkReadComponent.INSTANCE,
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
        );
    }

    MecanumDrive mecanumDrive = new MecanumDrive();
    private boolean turretToggle = false;
    private boolean turretLastButtonState = false;

    private final PolygonZone closeLaunchZone = new PolygonZone(new Point(144, 144), new Point(72, 72), new Point(0, 144));
    private final PolygonZone farLaunchZone = new PolygonZone(new Point(48, 0), new Point(72, 24), new Point(96, 0));
    private final PolygonZone blueBase = new PolygonZone(new Point(105.5, 33.5), 20, 20);
    private final PolygonZone redBase = new PolygonZone(new Point(38.5, 33.5), 20, 20);

    TimerEx endGame = new TimerEx(120);

    @Override
    public void onInit() {
        LimelightSubsystem.INSTANCE.setTeleOp(true);
    }

    @Override
    public void onStartButtonPressed() {
        mecanumDrive.start();
        LimelightSubsystem.INSTANCE.switchPipeline(1);
    }

    @Override
    public void onUpdate() {
        boolean currentButtonStateSquare = gamepad1.square;
        if (currentButtonStateSquare && !turretLastButtonState) {
            turretToggle = !turretToggle;
        }
        turretLastButtonState = currentButtonStateSquare;


        }
    }

