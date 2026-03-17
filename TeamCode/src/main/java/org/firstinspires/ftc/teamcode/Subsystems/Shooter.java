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
import dev.nextftc.hardware.impl.ServoEx;

@Configurable
public class Shooter implements Subsystem {
    public double velocity, currentVelocity, velocityAuto;
    public static double angulation;
    public static double velocityFix = 1220;
    // Coeficientes do Shooter
    private static PIDCoefficients coefficientsShooter = new PIDCoefficients(0.0001, 0, 0);
    private static BasicFeedforwardParameters feedforwardShooter = new BasicFeedforwardParameters(0.00023, 0, 0.1);

    // Instância da Limelight
    private LimelightSubsystem limelight = LimelightSubsystem.INSTANCE;
    private Led led = Led.INSTANCE;

    // Instância do Shooter
    public static final Shooter INSTANCE = new Shooter();

    private Shooter() {
    }

    // Configurações
    public boolean enabledTeleOp, enabledAuto = false;

    private MotorEx shooterMotor_Left = new MotorEx("shooter_motor_left", -1)
            .brakeMode();
    private MotorEx shooterMotor_Right = new MotorEx("shooter_motor_right", -1)
            .brakeMode();
    private MotorGroup shooterMotor = new MotorGroup(shooterMotor_Left, shooterMotor_Right);

    private ServoEx hood = new ServoEx("hood_servo", -1);

    // Sistemas de Controle (Shooter)
    private ControlSystem controlShooter = ControlSystem.builder()
            .velPid(coefficientsShooter)
            .basicFF(feedforwardShooter)
            .build();

    // Comandos do Subsistemai
    // Comandos Autonomo
    public Command shooterAutoOn() {
        enabledAuto = true;
        return new LambdaCommand()
                .setStart(() -> controlShooter.setGoal(new KineticState(0, velocityAuto, 0)))
                .setIsDone(() -> true);
    }

    // Comandos Teleop
    public Command shooterOn() {
        enabledTeleOp = true;
        return new RunToVelocity(controlShooter, velocity, new KineticState(0, velocity, Double.POSITIVE_INFINITY))
                .requires(this);
    }

    public Command fixedVelocity() {
        enabledTeleOp = true;
        return new RunToVelocity(controlShooter, velocityFix, new KineticState(0, velocityFix, Double.POSITIVE_INFINITY))
                .requires(this);
    }

    private double speedCalculationRed(double x) {
        double rpm = (((-2.86778e-6 * x + 5.02307e-4) * x
                + 0.0513528) * x
                - 6.02487) * x
                + 1280.38268;

        if (rpm > 1800.0) {
            rpm = 1800.0;
        } else if (rpm < 1150.0) {
            rpm = 1150.0;
        }

        return rpm;
    }

    private double setHoodRed(double x) {
        double pos = (((2.23363e-8 * x - 8.28222e-6) * x
                + 0.00106726) * x
                - 0.0506413) * x
                + 1.28864;

        if (pos > 1.0) {
            pos = 1.0;
        } else if (pos < 0.51) {
            pos = 0.51;
        }

        return pos;
    }

    private double setHoodBlue(double x) {
        double pos = (((1.71986e-8 * x - 6.58702e-6) * x
                + 0.000868147) * x
                - 0.040747) * x
                + 1.10966;

        if (pos > 1.0) {
            pos = 1.0;
        } else if (pos < 0.51) {
            pos = 0.51;
        }

        return pos;
    }

    private double speedCalculationBlue(double x) {
        double rpm = (((3.55109e-6 * x - 0.00195722) * x
                + 0.358762) * x
                - 20.36076) * x
                + 1488.38776;

        if (rpm > 1800.0) {
            rpm = 1800.0;
        } else if (rpm < 1150.0) {
            rpm = 1150.0;
        }

        return rpm;
    }

    // Comandos Autonomo
    public void switchVelocity(int velocity) {
        velocityAuto = velocity;
    }
    public void switchHood(double pos) {
        hood.setPosition(pos);
    }
    public void initMechanisms(boolean red) {
        if (red) {
            velocity = speedCalculationRed(testTurret.INSTANCE.movedDistance);
            angulation = setHoodRed(testTurret.INSTANCE.movedDistance);
        } else {
            velocity = speedCalculationBlue(testTurret.INSTANCE.movedDistance);
            angulation = setHoodBlue(testTurret.INSTANCE.movedDistance);
        }
        hood.setPosition(angulation);
    }

    public void testHood(double angulation) {
        hood.setPosition(angulation);
    }


    public void stopTeleOp() {
        enabledTeleOp = false;
    }

    public void stopAuto() {
        enabledAuto = false;
    }

    @Override
    public void initialize() {
        enabledTeleOp = false;
        enabledAuto = false;
    }

    // Rodando em looping assim que a programação iniciar
    @Override
    public void periodic() {
        if (enabledTeleOp || enabledAuto) {
            if (enabledTeleOp) {
                if (currentVelocity < velocityFix) {
                    shooterMotor.setPower(1);
                } else {
                    shooterMotor.setPower(controlShooter.calculate(shooterMotor.getState()));
                }
            }
            if (enabledAuto) {
                if (currentVelocity < velocityAuto) {
                    shooterMotor.setPower(1);
                } else {
                    shooterMotor.setPower(controlShooter.calculate(shooterMotor.getState()));
                }
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
