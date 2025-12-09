package org.firstinspires.ftc.teamcode.Mecanismos;

import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.impl.MotorEx;

public class MecanumDrive {
    private final MotorEx frontLeftMotor = new MotorEx("front_left").brakeMode(); // Port 1
    private final MotorEx frontRightMotor = new MotorEx("front_right").reversed().brakeMode(); // Port 2
    private final MotorEx backLeftMotor = new MotorEx("back_left").reversed().brakeMode(); // Port 0
    private final MotorEx backRightMotor = new MotorEx("back_right").brakeMode(); // Port 3

    public void start() {
        DriverControlledCommand driverControlled = new PedroDriverControlled(
                Gamepads.gamepad1().leftStickY().negate(),
                Gamepads.gamepad1().leftStickX().negate(),
                Gamepads.gamepad1().rightStickX().negate(),
                true
        );
        driverControlled.schedule();
    }
}
