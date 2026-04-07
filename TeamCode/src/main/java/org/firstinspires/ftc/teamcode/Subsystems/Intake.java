package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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
import dev.nextftc.hardware.positionable.SetPosition;

@Configurable
public class Intake implements Subsystem {

    // Coeficientes do PID
    public static PIDCoefficients coefficients = new PIDCoefficients(0.001, 0, 0);
    // Coeficientes do FeedForward
    private static BasicFeedforwardParameters feedforward = new BasicFeedforwardParameters(0.00046, 0, 0);
    public static double velocityAuto = 3000;
    public static double velocity = 3000;
    public double currentVelocity;

    // Instância do Intake
    public static final Intake INSTANCE = new Intake();
    private Intake() {}

    // Configurações
    private boolean enabled = false;

    private ServoEx lock = new ServoEx("lock_servo", -1); // Port
    private MotorEx motor_left = new MotorEx("intake_motor_left", -1) // Port
            .floatMode();

    private MotorEx motor_right = new MotorEx("intake_motor_right", -1) // Port
            .reversed()
            .floatMode();

    private MotorGroup intakeMotor = new MotorGroup(motor_left, motor_right);


    private ControlSystem controlSystem = ControlSystem.builder()
            .basicFF(feedforward)
            .velPid(coefficients)
            .build();

    // Comandos do Subsistema
    // Comandos Autonomo
    public Command coletAutoOn() {
        enabled = true;
        return new LambdaCommand()
                .setStart(() -> controlSystem.setGoal(new KineticState(0, velocityAuto, 0)))
                .setIsDone(() -> true);

    }

    public Command stopAuto() {
        return new LambdaCommand()
                .setStart(() -> controlSystem.setGoal(new KineticState(0, 0 ,0)))
                .setIsDone(() -> currentVelocity <= 100);
    }

    // Comandos Teleop
    public Command colet() {
        enabled = true;
        return new RunToVelocity(controlSystem, velocity).requires(this);
    }

    public Command eject() {
        enabled = true;
        return new RunToVelocity(controlSystem, -2000).requires(this);
    }

    public Command locked = new SetPosition(lock, 0).requires(this);

    public Command unlocked = new SetPosition(lock, 0.5);

    public void stop() {
        enabled = false;
    }

    // Roda em looping infinito assim que a programação iniciar
    @Override
    public void initialize() {
        enabled = false;
    }

    @Override
    public void periodic() {
        if (enabled) {
            intakeMotor.setPower(controlSystem.calculate(intakeMotor.getState()));
        } else {
            intakeMotor.setPower(0);
        }

        currentVelocity = intakeMotor.getVelocity();

    }
}
