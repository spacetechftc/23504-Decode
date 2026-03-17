package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class LimelightSubsystem implements Subsystem {
    private Limelight3A limelight;
    public static LimelightSubsystem INSTANCE = new LimelightSubsystem();
    private LimelightSubsystem() {}
    public double tx, ta, distance;
    public int id;
    public boolean track;

    // Reset do ID inicial (Usar em autônomo)
    public Command reset = new LambdaCommand()
            .setStart(() -> {
                id = 0;
            });

    // Troca do ID para o GOAL Vermelho
    public Command switchIndexRed = new LambdaCommand()
            .setUpdate(() -> {
                limelight.pipelineSwitch(1);
            });

    // Troca do ID para o GOAL Azul
    public Command switchIndexBlue = new LambdaCommand()
            .setUpdate(() -> {
                limelight.pipelineSwitch(2);
            });

    // Cálculo da distância do robô em relação ao GOAL
    public double getDistancefromTag(double ta) {
        double distance = Math.pow(ta/4167.12, 1/-1.59416);
        return distance / 100;

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
    }

    // Roda em looping inifinito assim que a programação iniciar.
    @Override
    public void periodic() {
        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            tx = result.getTx(); // Quão longe à esquerda ou à direita está o alvo (graus)
            ta = result.getTa(); // Quão grande o alvo parece (0%-100% da imagem)
            distance = getDistancefromTag(ta);
            track = true;
        } else {
            track = false;
        }

        Pose3D botPose = result.getBotpose();
        double poseX = (botPose.getPosition().x * 39.3700787) - 72; // X vai pro Y
        double poseY = (botPose.getPosition().y * 39.3700787) + 72; // Y vai pro X
    }


}