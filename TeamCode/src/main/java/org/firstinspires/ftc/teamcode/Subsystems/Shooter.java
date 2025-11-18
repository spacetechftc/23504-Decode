package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class Shooter implements Subsystem {
    public double velocity, speedCalculation, currentVelocity;
    public static double velocityz = 1200;

    // Coeficientes do Shooter
    private static PIDCoefficients coefficientsShooter = new PIDCoefficients(0.023, 0, 0);
    private static BasicFeedforwardParameters feedforwardShooter = new BasicFeedforwardParameters(0.0005, 0, 0);

    // Instância da Limelight
    private LimelightSubsystem limelight = LimelightSubsystem.INSTANCE;

    // Instância do Shooter
    public static final Shooter INSTANCE = new Shooter();

    private Shooter() {
    }

    // Configurações
    public boolean enabled = false;

    private MotorEx shooterMotor_Left = new MotorEx("shooter_motor_left")
            .brakeMode();
    private MotorEx shooterMotor_Right = new MotorEx("shooter_motor_right")
            .brakeMode();
    private MotorGroup shooterMotor = new MotorGroup(shooterMotor_Left, shooterMotor_Right);

    // Sistemas de Controle (Shooter)
    private ControlSystem controlShooter = ControlSystem.builder()
            .velPid(coefficientsShooter)
            .basicFF(feedforwardShooter)
            .build();


    // Comandos do Subsistema
        // Cálculo da velocidade ideal
    public double controlVelocity(double gravity, double distance, double angle, double height) {
        double thetaRad = Math.toRadians(angle);
        double numerador = gravity * Math.pow(distance, 2);
        double cosTheta = Math.cos(thetaRad);
        double tanTheta = Math.tan(thetaRad);
        double denominador = 2 * Math.pow(cosTheta, 2) * (distance * tanTheta - height);
        double velocity = Math.sqrt(numerador / denominador);
        return velocity;
    }

        // Conversor de M/S para TPS
    public double velocityToTPS(double velocity) {
        double raio = 0.0044;
        return (((velocity / raio) / (2 * Math.PI)) * 28) * 0.24;
    }
    // Comandos Autonomo
    public Command shooterAutoOn() {
        enabled = true;
        return new LambdaCommand()
                .setStart(() -> controlShooter.setGoal(new KineticState(0, 1100, 0)))
                .setIsDone(() -> true);
    }

    public Command shooterAutoNegative() {
        enabled = true;
        return new LambdaCommand()
                .setStart(() -> controlShooter.setGoal(new KineticState(0, -1200, 0)))
                .setIsDone(() -> true);
    }

    public Command stopAuto() {
        return new LambdaCommand()
                .setStart(() -> shooterMotor.setPower(0))
                .setIsDone(() -> true);
    }

    // Comandos Teleop
    public Command shooterOn() {
        enabled = true;
        return new RunToVelocity(controlShooter, velocity, new KineticState(0, velocity, Double.POSITIVE_INFINITY))
                .requires(this);
    }

    public Command shooterNegative() {
        enabled = true;
        return new RunToVelocity(controlShooter, -1200, new KineticState(0, -1200, Double.POSITIVE_INFINITY))
                .requires(this);

    }

    public void stop() {
        enabled = false;
    }

    @Override
    public void initialize() {
        enabled = false;
    }

    // Rodando em looping assim que a programação iniciar
    @Override
    public void periodic() {
        // Controle da velocidade Ideal
        speedCalculation = controlVelocity(9.81, limelight.distance, 65, 0.6);
        velocity = velocityToTPS(speedCalculation);
        if (velocity > 1600) {
            velocity = 1600;
        }

        if (enabled) {
            shooterMotor.setPower(controlShooter.calculate(shooterMotor.getState()));
        } else {
            shooterMotor.setPower(0.001);
        }

        currentVelocity = shooterMotor.getVelocity();

    }
}
