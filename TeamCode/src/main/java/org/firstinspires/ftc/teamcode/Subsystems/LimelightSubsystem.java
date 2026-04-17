package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.ServoEx;
import dev.nextftc.hardware.positionable.SetPosition;
import dev.nextftc.hardware.positionable.SetPositions;
import kotlinx.coroutines.channels.ActorKt;

@Configurable
public class LimelightSubsystem implements Subsystem {
    private static Limelight3A limelight;
    public static LimelightSubsystem INSTANCE = new LimelightSubsystem();
    private LimelightSubsystem() {}
    public double tx, ta;
    public double INCHES_PER_METER = 39.3700787;
    public static double FIELD_CENTER = 63;
    public static double xOffset = -4;
    public static double yOffset = 5;

    public static double headingOffset = -20;

    public boolean autonomous, teleOp;
    public double pedroX, pedroY, headingRadians;

    private ServoEx limelight_servo = new ServoEx("limelight_servo", -1);

    public Command relocalization = new SetPosition(limelight_servo, 0.9).requires(this);

    public Command artifact = new SetPosition(limelight_servo, 0).requires(this);

    public void switchPipeline(int id) {
        limelight.pipelineSwitch(id);
    }

    public static Pose getRobotPoseMT1() {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid() || result.getBotpose() == null) {
            return null;
        }

        Pose3D botPose = result.getBotpose();

        double xInches = botPose.getPosition().x * INSTANCE.INCHES_PER_METER;
        double yInches = botPose.getPosition().y * INSTANCE.INCHES_PER_METER;

        double headingRadians = Math.toRadians(botPose.getOrientation().getYaw(AngleUnit.DEGREES) - 90);

        double pedroX = -xInches + FIELD_CENTER - xOffset;
        double pedroY = yInches + FIELD_CENTER - yOffset;

        return new Pose(pedroX, pedroY, headingRadians);
    }

    public double getP() {
        double strafe = -tx * 0.013;

        if (Math.abs(tx) < 0.5) {
            strafe = 0;
        }

        return Math.max(-0.7, Math.min(0.7, strafe));
    }

    public void setTeleOp(boolean setter) {
        teleOp = setter;
        if (setter) autonomous = false;
    }

    public void setAutonomous(boolean setter) {
        autonomous = setter;
        if (setter) teleOp = false;
    }

    public boolean hasTarget() {
        return ta > 0;
    }

    // Inicialização da Limelight
    @Override
    public void initialize() {
        if (limelight == null) {
            limelight = ActiveOpMode.hardwareMap()
                    .get(Limelight3A.class, "limelight");
            limelight.setPollRateHz(100);
            limelight.start();
        }
        tx = 0;
    }

    // Roda em looping inifinito assim que a programação iniciar.
    @Override
    public void periodic() {
        if (autonomous) {
            // Irá rodar no autonomo o tempo todo
            LLResult result = limelight.getLatestResult();
            if (result != null && result.isValid()) {
                tx = result.getTx(); // Quão longe à esquerda ou direita o alvo está (graus)
                ta = result.getTa();

                ActiveOpMode.telemetry().addData("Alvo X", tx);
            } else {
                ActiveOpMode.telemetry().addData("Limelight", "Sem Alvos");
            }

        } else if (teleOp) {
            // Irá rodar no teleop o tempo todo
            LLResult result = limelight.getLatestResult();

            if (result == null || !result.isValid() || result.getBotpose() == null) {
                return;
            }
            Pose3D botPose = result.getBotpose();

            double xInches = botPose.getPosition().x * INSTANCE.INCHES_PER_METER;
            double yInches = botPose.getPosition().y * INSTANCE.INCHES_PER_METER;

            headingRadians = Math.toRadians(botPose.getOrientation().getYaw(AngleUnit.DEGREES) - 90 - headingOffset);

            pedroY = -xInches + FIELD_CENTER - yOffset;
            pedroX = yInches + FIELD_CENTER - xOffset;

        }
    }
}

