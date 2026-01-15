package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.control.feedback.PIDCoefficients;
import dev.nextftc.control.feedforward.BasicFeedforwardParameters;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.controllable.RunToVelocity;
import dev.nextftc.hardware.impl.MotorEx;

@Configurable
public class Intake implements Subsystem {
    private ColorRangeSensor sensor;
    public double distance;
    // Coeficientes do PID
    public static PIDCoefficients coefficients = new PIDCoefficients(0.001, 0, 0);
    // Coeficientes do FeedForward
    private static BasicFeedforwardParameters feedforward = new BasicFeedforwardParameters(0.00046, 0, 0);
    public static double velocity = 2000;
    public double currentVelocity;

    // Instância do Intake
    public static final Intake INSTANCE = new Intake();
    private Intake() {}

    // Configurações
    private boolean enabled = false;

    private MotorEx motor = new MotorEx("intake_motor")
            .floatMode();

    private ControlSystem controlSystem = ControlSystem.builder()
            .basicFF(feedforward)
            .velPid(coefficients)
            .build();

    // Comandos do Subsistema
    // Comandos Autonomo
    public Command coletAutoOn() {
        enabled = true;
        return new LambdaCommand()
                .setStart(() -> controlSystem.setGoal(new KineticState(0, velocity, 0)))
                .setIsDone(() -> true);

    }
    public Command stopAuto() {
        return new LambdaCommand()
                .setStart(() -> controlSystem.setGoal(new KineticState(0, 0 ,0)))
                .setIsDone(() -> true);
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

    public void stop() {
        enabled = false;
    }

    // Roda em looping infinito assim que a programação iniciar
    @Override
    public void initialize() {
        sensor = ActiveOpMode.hardwareMap().get(ColorRangeSensor.class,"sensor");
        distance = sensor.getDistance(DistanceUnit.MM);
        enabled = false;
    }

    @Override
    public void periodic() {
        if (enabled) {
            motor.setPower(controlSystem.calculate(motor.getState()));
        } else {
            motor.setPower(0);
        }
        currentVelocity = motor.getVelocity();

    }
}
