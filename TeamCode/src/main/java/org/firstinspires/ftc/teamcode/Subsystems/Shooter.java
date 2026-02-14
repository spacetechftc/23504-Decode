package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.MotorGroup;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class Shooter implements Subsystem {
    public double velocity, currentVelocity, angulation;
    public static double velocityFix = 1220;
    public Servo hood;
    // Coeficientes do Shooter
    private static PIDCoefficients coefficientsShooter = new PIDCoefficients(0.023, 0, 0);
    private static BasicFeedforwardParameters feedforwardShooter = new BasicFeedforwardParameters(0.00051, 0, 0.1);

    // Instância da Limelight
    private LimelightSubsystem limelight = LimelightSubsystem.INSTANCE;
    private Led led = Led.INSTANCE;

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

    // Comandos do Subsistemai
    // Comandos Autonomo
    public Command shooterAutoOn() {
        enabled = true;
        return new LambdaCommand()
                .setStart(() -> controlShooter.setGoal(new KineticState(0, 1140, 0)))
                .setIsDone(() -> true);
    }

    public Command shooterAutoOnFar() {
        enabled = true;
        return new LambdaCommand()
                .setStart(() -> controlShooter.setGoal(new KineticState(0, 1600, 0)))
                .setIsDone(() -> true);
    }

    public Command shooterAutoNegative() {
        enabled = true;
        return new LambdaCommand()
                .setStart(() -> controlShooter.setGoal(new KineticState(0, -1200, 0)))
                .setIsDone(() -> true);
    }

    // Comandos Teleop
    public Command shooterOn() {
        enabled = true;
        return new RunToVelocity(controlShooter, velocity, new KineticState(0, velocity, Double.POSITIVE_INFINITY))
                .requires(this);
    }

    public void shooterOAn() {
        enabled = true;
        controlShooter.setGoal(new KineticState(0, velocity, 0));
    }

    public Command fixedVelocity() {
        enabled = true;
        return new RunToVelocity(controlShooter, velocityFix, new KineticState(0, velocityFix, Double.POSITIVE_INFINITY))
                .requires(this);
    }

    public Command shooterNegative() {
        enabled = true;
        return new RunToVelocity(controlShooter, -1200, new KineticState(0, -1200, Double.POSITIVE_INFINITY))
                .requires(this);

    }

    private double speedCalculation(double x) {
        double rpm = (((0.00000347275 * x - 0.00167072) * x
                + 0.294964) * x
                - 15.54368) * x
                + 1388.88034;

        if (rpm > 1800.0) {
            rpm = 1800.0;
        } else if (rpm < 1150.0) {
            rpm = 1150.0;
        }

        return rpm;
    }

    private double setHood(double x) {
        double pos = (((8.53931e-9 * x - 3.28723e-6) * x
                + 4.28963e-4) * x
                - 0.0171724) * x
                + 0.711223;

        if (pos > 1.0) {
            pos = 1.0;
        } else if (pos < 0.51) {
            pos = 0.51;
        }

        return pos;
    }

    public void stop() {
        enabled = false;
    }

    @Override
    public void initialize() {
        hood = ActiveOpMode.hardwareMap().get(Servo.class, "hood_servo");
        enabled = false;
    }

    // Rodando em looping assim que a programação iniciar
    @Override
    public void periodic() {
        velocity = speedCalculation(testTurret.INSTANCE.movedDistance);
        angulation = setHood(testTurret.INSTANCE.movedDistance);

        hood.setPosition(angulation);

        if (enabled) {
            if (currentVelocity < velocity) {
                shooterMotor.setPower(1);
            } else {
                shooterMotor.setPower(controlShooter.calculate(shooterMotor.getState()));
            }
        } else {
            shooterMotor.setPower(0);
        }

        currentVelocity = shooterMotor.getVelocity();
        if (currentVelocity >= velocity - 20 && currentVelocity <= velocity + 200) {
            Led.INSTANCE.setLed_shooter(0.85);
        } else {
            Led.INSTANCE.setLed_shooter(0);
        }

    }
}
