package org.firstinspires.ftc.teamcode.Subsystems;

import static org.firstinspires.ftc.teamcode.opModes.tests.testGraph.telemetryMa;

import com.bylazar.configurables.annotations.Configurable;
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
    private Led led = Led.INSTANCE;

    private ServoEx lock = new ServoEx("lock_servo", -1); // Port
    private MotorEx motor = new MotorEx("intake_motor", -1)// Port
            .reversed()
            .floatMode();
    private DistanceSensor sensor_up;
    private DistanceSensor sensor_mid;
    private DistanceSensor sensor_down;

    private ControlSystem controlSystem = ControlSystem.builder()
            .basicFF(feedforward)
            .velPid(coefficients)
            .build();

    // Comandos do Subsistema

    public void updateArtifactLed(boolean sensor1Detected, boolean sensor2Detected, boolean sensor3Detected) {
        int count = 0;

        if (sensor1Detected) count++;
        if (sensor2Detected) count++;
        if (sensor3Detected) count++;

        switch (count) {
            case 0:
                led.setBackOff();
                break;
            case 1:
                led.setBackRed();
                break;
            case 2:
                led.setBackYellow();
                break;
            case 3:
                led.setBackGreen();
                break;
        }
    }
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
    public Command locked = new SetPosition(lock, 1.0).requires(this);
    public Command unlocked = new SetPosition(lock, 0);

    public void stop() {
        enabled = false;
    }

    // Roda em looping infinito assim que a programação iniciar
    @Override
    public void initialize() {
        enabled = false;

        sensor_up = ActiveOpMode.hardwareMap().get(DistanceSensor.class, "sensor_up");
        sensor_mid = ActiveOpMode.hardwareMap().get(DistanceSensor.class, "sensor_mid");
        sensor_down = ActiveOpMode.hardwareMap().get(DistanceSensor.class, "sensor_down");



    }

    @Override
    public void periodic() {
        currentVelocity = motor.getVelocity();

        if (enabled) {
            motor.setPower(controlSystem.calculate(motor.getState()));
        } else {
            motor.setPower(0);
        }

        boolean sensor1Detected = sensor_up.getDistance(DistanceUnit.CM) < 3.7;
        boolean sensor2Detected = sensor_mid.getDistance(DistanceUnit.CM) < 3.2;
        boolean sensor3Detected = sensor_down.getDistance(DistanceUnit.CM) < 7.3;

        updateArtifactLed(sensor1Detected, sensor2Detected, sensor3Detected);


    }
}
