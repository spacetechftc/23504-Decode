package org.firstinspires.ftc.teamcode.Mecanismos;

import org.firstinspires.ftc.teamcode.Subsystems.WebcamSubsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
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

    public Command strafeToBallCommand() {
        return new LambdaCommand()
                .setStart(() -> {
                    // Opcional: zerar algum estado quando começar
                })
                .setUpdate(() -> {
                    if (Math.abs(WebcamSubsystem.INSTANCE.x) < 20) {
                        goToBalls(0);
                    } else {
                        double strafe = WebcamSubsystem.INSTANCE.getPID();
                        goToBalls(strafe);
                    }
                })
                .setIsDone(() ->
                        Math.abs(WebcamSubsystem.INSTANCE.x) < 20
                )
                .requires(this)
                .named("StrafeToBall");
    }

    public void goToBalls(double strafe) {
        frontLeftMotor.setPower(0 + strafe + 0);
        backLeftMotor.setPower(0 - strafe + 0);
        frontRightMotor.setPower(0 - strafe - 0);
        backRightMotor.setPower(0 + strafe - 0);

    }

}
