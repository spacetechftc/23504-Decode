package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Mecanismos.PIDFController;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;

@Configurable
public class testTurret implements Subsystem {

    PIDFController odometryTicksControl = new PIDFController(KP, 0, KD, 0);
    PIDFController limelightControl = new PIDFController(kP, 0, kD, 0);
    DcMotorEx turretMotor;
    public int currentTicks, targetTicks;

    // Configuração do PID -- Limelight
    public static double kP = 0.016;
    public static double kD = 0;

    // Configuração do PID -- Odometria
    public static double KP = 0.00026;
    public static double KD = 0.00001;

    // Ângulo da Torreta
    public static double GEAR_RATIO = 83.0/27.0; // Your gear ratio
    private static final double ENCODER_CPR = 8192;
    private static final double TICKS_PER_TURRET_REV = ENCODER_CPR * GEAR_RATIO;

    // LIMITES DO ENCODER
    private double LEFT_LIMIT = 5000;
    private double RIGHT_LIMIT = -5000;

    // GOALS
    public static double blueGoalX = 0;
    public static double blueGoalY = 144;
    public static double redGoalX  = 138;
    public static double redGoalY  = 113;
    public double power = 0;

    // Instância da Torreta
    public static final testTurret INSTANCE = new testTurret();
    private testTurret() {}

    // Controle baseado na odometria
    private int degreesToTicks(double degrees) {
        return (int) Math.round((degrees / 360.0) * TICKS_PER_TURRET_REV);
    }
    private double ticksToDegrees(int ticks) {
        return (ticks / TICKS_PER_TURRET_REV) * 360.0;
    }

    public Command resetEncoder = new LambdaCommand()
            .setUpdate(() -> {
                turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            });

    public double alignTurretWithOdometry(double x, double y, double heading, int currentTicks, boolean blue) {
        double headingDeg = Math.toDegrees(heading);

        double goalX = blue ? blueGoalX : redGoalX;
        double goalY = blue ? blueGoalY : redGoalY;

        double angleToGoal = Math.toDegrees(Math.atan2(goalY - y, goalX - x));

        double turretAngle = normalizeAngle(headingDeg - angleToGoal);

        targetTicks = degreesToTicks(turretAngle);
        power = odometryTicksControl.calculate(targetTicks, currentTicks);
        power = Math.max(-1, Math.min(1, power)); // Clamp power to [-1, 1]

        // ----- Limites Fisicos -----
        // Bloqueia para a esquerda
        if (currentTicks >= LEFT_LIMIT && power < 0) {
            power = 0;
        }

        // Bloqueia para a direita
        if (currentTicks <= RIGHT_LIMIT && power > 0) {
            power = 0;
        }

        return power;
    }

    // ----- Controle PID baseado na limelight -----
    public double alignTurretWithLimelight() {
        double target = 0; // graus

        double output = limelightControl.calculate(target, LimelightSubsystem.INSTANCE.tx);
        output = Math.max(-1, Math.min(1, output)); // Clamp power to [-1, 1]

        return output;
    }

    public void alignTurret(double x, double y, double heading, int currentTicks, boolean blue) {
        if (LimelightSubsystem.INSTANCE.track) {
            power = alignTurretWithLimelight();
        } else {
            power = alignTurretWithOdometry(x, y, heading, currentTicks, blue);
        }

        // Bloqueia para a esquerda
        if (currentTicks >= LEFT_LIMIT && power < 0) {
            power = 0;
        }

        // Bloqueia para a direita
        if (currentTicks <= RIGHT_LIMIT && power > 0) {
            power = 0;
        }

        turretMotor.setPower(power);
    }

    public void manualTurret(boolean left, boolean right) {
        if (left) {
            turretMotor.setPower(0.6);
        } else if (right) {
            turretMotor.setPower(-0.6);
        } else {
            turretMotor.setPower(0);
        }
    }

    public void turretToPosition(int pos) {
        targetTicks = pos;
        power = odometryTicksControl.calculate(targetTicks, currentTicks);
        power = Math.max(-1, Math.min(1, power));
        turretMotor.setPower(power);
    }

    private double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }

    @Override
    public void initialize() {
        turretMotor = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "turret_motor");
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void periodic() {
        currentTicks = turretMotor.getCurrentPosition();
        ActiveOpMode.telemetry().addData("Turret", "------------------");
        ActiveOpMode.telemetry().addData("CurrentTicks", currentTicks);
        ActiveOpMode.telemetry().addData("TargetTicks", targetTicks);

        PedroComponent.follower().update();
    }
}