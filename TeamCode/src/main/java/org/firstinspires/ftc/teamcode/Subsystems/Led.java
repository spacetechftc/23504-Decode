package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

@Configurable
public class Led implements Subsystem {

    public Servo led_shooter; // Port 1
    public Servo led_back; // Port 0

    public static double pos;
    // 0.279 RED
    // 0.5 GREEN
    // 0.612 BLIE

    public static final Led INSTANCE = new Led();
    private Led() {}

    public void setLedBack(double pos) {
        led_back.setPosition(pos);
    }

    public void setLed_shooter(double pos) {
        led_shooter.setPosition(pos);
    }

    @Override
    public void initialize() {
        led_back = ActiveOpMode.hardwareMap().get(Servo.class, "led_back");
        led_shooter = ActiveOpMode.hardwareMap().get(Servo.class, "led_shooter");
    }

    @Override
    public void periodic() {
    }
}
