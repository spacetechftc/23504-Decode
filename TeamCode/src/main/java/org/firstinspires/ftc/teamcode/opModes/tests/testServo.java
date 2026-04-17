package org.firstinspires.ftc.teamcode.opModes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "ServoTestDashboard", group = "OpModes Tests")
public class testServo extends OpMode {

    public static double servoPosLeft = 0;
    public static double servoPosRight = 0;

    Servo servo;
    Servo servox;

    @Override
    public void init() {
        servo = hardwareMap.servo.get("endgame_left");
        servo.setDirection(Servo.Direction.REVERSE);
        servox = hardwareMap.servo.get("endgame_right");
    }

    @Override
    public void loop() {
        servo.setPosition(servoPosLeft);
        servox.setPosition(servoPosRight);
        telemetry.addData("Servo position Left", servoPosLeft);
        telemetry.addData("Servo position Right", servoPosRight);
        telemetry.update();
    }
}