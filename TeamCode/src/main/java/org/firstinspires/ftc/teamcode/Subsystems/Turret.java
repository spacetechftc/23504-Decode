package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.impl.CRServoEx;

@Configurable
public class Turret implements Subsystem {

    DcMotorEx turretEncoder;
    public int currentTicks;
    public int targetTicks;

    // Configuração do PID -- Limelight
    public static double kP = 0.02;
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
    private double LEFT_LIMIT = 5800;
    private double RIGHT_LIMIT = -5500;

    // GOALS
    public static double blueGoalX = 12;
    public static double blueGoalY = 130;
    public static double redGoalX  = 132;
    public static double redGoalY  = 130;

    // Configuração do Servo (Rotação Contínua)
    private CRServoEx servo = new CRServoEx("turret_servo");

    // Instância da Torreta
    public static final Turret INSTANCE = new Turret();
    private Turret() {}

    // Controle baseado na odometria

    private int degreesToTicks(double degrees) {
        return (int) Math.round((degrees / 360.0) * TICKS_PER_TURRET_REV);
    }
    private double ticksToDegrees(int ticks) {
        return (ticks / TICKS_PER_TURRET_REV) * 360.0;
    }

    public double turretToPosition(int targetTicks, int currentTicks) {
        int error = currentTicks - targetTicks;
        double derivative = error - LASTERROR;
        double power = (KP * error) + (KD * derivative);

        if (currentTicks >= LEFT_LIMIT && power < 0) {
            power = 0;
        }
        if (currentTicks <= RIGHT_LIMIT && power > 0) {
            power = 0;
        }

        LASTERROR = error;
        ActiveOpMode.telemetry().addData("Turret", "------------------");
        ActiveOpMode.telemetry().addData("CurrentTicks", currentTicks);
        ActiveOpMode.telemetry().update();
        return power;
    }

    public double alignTurretWithOdometry(double x, double y, double heading, int currentTicks) {
        double headingDeg = Math.toDegrees(heading);

        double goalX = redGoalX;
        double goalY = redGoalY;

        double angleToGoal = Math.toDegrees(Math.atan2(goalY - y, goalX - x));

        double turretAngle = angleToGoal + headingDeg - 16;

        targetTicks = degreesToTicks(turretAngle);
        int error = currentTicks - targetTicks;
        double derivative = error - LASTERROR;
        double power = (KP * error) + (KD * derivative);

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

    @Override
    public void initialize() {
        turretEncoder = ActiveOpMode.hardwareMap().get(DcMotorEx.class,"back_left");
        turretEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void periodic() {

        double x = PedroComponent.follower().getPose().getX();
        double y = PedroComponent.follower().getPose().getY();
        double heading = PedroComponent.follower().getPose().getHeading();
        currentTicks = turretEncoder.getCurrentPosition();

        ActiveOpMode.telemetry().addData("Odometria", "------------------");
        ActiveOpMode.telemetry().addData("X", PedroComponent.follower().getPose().getX());
        ActiveOpMode.telemetry().addData("Y", PedroComponent.follower().getPose().getY());
        ActiveOpMode.telemetry().addData("Heading", PedroComponent.follower().getPose().getHeading());
        ActiveOpMode.telemetry().addData("Turret", "------------------");
        ActiveOpMode.telemetry().addData("CurrentTicks", currentTicks);
        ActiveOpMode.telemetry().addData("TargetTicks", targetTicks);

        double power;

        // Controle com Limelight
        if (LimelightSubsystem.INSTANCE.track) {
            power = alignTurretWithLimelight();
        } else {
            power = alignTurretWithOdometry(x, y, heading, currentTicks);
        }

        // ----- Limites Fisicos -----
        // Bloqueia para a esquerda
        if (currentTicks >= LEFT_LIMIT && power < 0) {
            power = 0;
        }

        // Bloqueia para a direita
        if (currentTicks <= RIGHT_LIMIT && power > 0) {
            power = 0;
        }
        servo.setPower(power);

    }
}