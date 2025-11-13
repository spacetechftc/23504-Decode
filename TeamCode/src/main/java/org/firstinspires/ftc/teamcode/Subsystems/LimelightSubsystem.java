package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.Mecanismos.AutoSelect;

import java.util.Arrays;
import java.util.List;

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
    public String verify;
    public boolean track;
    AutoSelect autoSelect = new AutoSelect();

    // Detecção do MOTIF
    public Command detectionID = new LambdaCommand()
            .setStart(() -> {
                limelight.pipelineSwitch(0);
                LLResult result = limelight.getLatestResult();
                List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
                for (LLResultTypes.FiducialResult fiducial : fiducials) {
                    id = fiducial.getFiducialId();
                    switch (id) {
                        case 21:
                            autoSelect.Pattern[0] = 'V'; autoSelect.Pattern[1] = 'R'; autoSelect.Pattern[2] = 'R';
                            verify = Arrays.toString(autoSelect.Pattern);
                            break;
                        case 22:
                            autoSelect.Pattern[0] = 'R'; autoSelect.Pattern[1] = 'V'; autoSelect.Pattern[2] = 'R';
                            verify = Arrays.toString(autoSelect.Pattern);
                            break;
                        case 23:
                            autoSelect.Pattern[0] = 'R'; autoSelect.Pattern[1] = 'R'; autoSelect.Pattern[2] = 'V';
                            verify = Arrays.toString(autoSelect.Pattern);
                            break;
                    }
                }
            });

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
    }


}