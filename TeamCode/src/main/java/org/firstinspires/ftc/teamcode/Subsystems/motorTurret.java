package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.TouchSensor;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.CRServoEx;

@Configurable
public class motorTurret implements Subsystem {
    DcMotorEx turretEncoder;
    DcMotorEx turretMotor;
    public int currentTicks;
    public int targetTicks;

    // Configuração do PID -- Limelight
    public static double kP = 0.016;
    public static double kD = 0;
    public static double kI = 0;
    private double integralSum = 0.0;
    private double lastError = 0.0;

    // Configuração do PID -- Odometria
    public static double KP = 0.00026;
    public static double KD = 0.00001;
    private double LASTERROR = 0.0;

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
    public static final motorTurret INSTANCE = new motorTurret();
    private motorTurret() {}

    // Controle baseado na odometria

    private int degreesToTicks(double degrees) {
        return (int) Math.round((degrees / 360.0) * TICKS_PER_TURRET_REV);
    }
    private double ticksToDegrees(int ticks) {
        return (ticks / TICKS_PER_TURRET_REV) * 360.0;
    }

    public void turretToMid() {
        targetTicks = 0;
        int error = currentTicks - targetTicks;
        double derivative = error - LASTERROR;
        double power = (KP * error) + (KD * derivative);

        LASTERROR = error;
        turretMotor.setPower(power);
    }

    private double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }

    public double alignTurretWithOdometry(double x, double y, double heading, int currentTicks, boolean blue) {
        double headingDeg = Math.toDegrees(heading);

        double goalX = blue ? blueGoalX : redGoalX;
        double goalY = blue ? blueGoalY : redGoalY;

        double angleToGoal = Math.toDegrees(Math.atan2(goalY - y, goalX - x));

        double turretAngle = blue ? normalizeAngle(headingDeg - angleToGoal) : headingDeg - angleToGoal;

        targetTicks = degreesToTicks(turretAngle);
        int error = currentTicks - targetTicks;
        double derivative = error - LASTERROR;
        double power = (KP * error) + (KD * derivative);

        // ----- Limites Fisicos -----
        // Bloqueia para a esquerda
        if (currentTicks >= LEFT_LIMIT && power < 0) {
            power = 0;
        }

        // Bloqueia para a direita
        if (currentTicks <= RIGHT_LIMIT && power > 0) {
            power = 0;
        }

        LASTERROR = error;
        return power;
    }

    // ----- Controle PID baseado na limelight -----
    public double alignTurretWithLimelight() {
        double deadband = 0.2; // graus

        // erro = centro (0) - deslocamento da limelight
        double error = 0 - LimelightSubsystem.INSTANCE.tx;

        if (Math.abs(error) < deadband) {
            integralSum = 0;
            return 0;
        }

        integralSum += error;

        double derivative = error - lastError;
        lastError = error;

        // PID completo
        double output = (kP * error) + (kI * integralSum) + (kD * derivative);

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

    public Command resetEncoder = new LambdaCommand()
            .setUpdate(() -> {
                turretEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            });


    @Override
    public void initialize() {
        turretMotor = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "turret_motor");
        turretEncoder = ActiveOpMode.hardwareMap().get(DcMotorEx.class,"back_left");
        turretEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void periodic() {
        currentTicks = turretEncoder.getCurrentPosition();
        ActiveOpMode.telemetry().addData("Turret", "------------------");
        ActiveOpMode.telemetry().addData("CurrentTicks", currentTicks);
        ActiveOpMode.telemetry().addData("TargetTicks", targetTicks);

        PedroComponent.follower().update();
    }
}