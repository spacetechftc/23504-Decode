package org.firstinspires.ftc.teamcode.opModes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Configurable
@TeleOp(name = "ServoTestDashboard")
public class testServo extends OpMode {

    public static double servoPos = 0;
    Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.servo.get("servo");
    }

    @Override
    public void loop() {
        servo.setPosition(servoPos);
        telemetry.addData("Servo position", servoPos);
    }
}