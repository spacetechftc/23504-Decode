package org.firstinspires.ftc.teamcode.opModes.tests;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="maxVelocity", group = "OpModes Tests")
public class MaxVelocityTest extends LinearOpMode {
    DcMotorEx motor;
    DcMotorEx motor2;
    double currentVelocity;
    double maxVelocity = 0.0;

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "shooter_motor_left");
        motor.setDirection(DcMotorSimple.Direction.REVERSE);
        motor2 = hardwareMap.get(DcMotorEx.class, "shooter_motor_right");
        waitForStart();
        while (opModeIsActive()) {
            motor.setPower(1);
            motor2.setPower(1);
            currentVelocity = motor.getVelocity();

            if (currentVelocity > maxVelocity) {
                maxVelocity = currentVelocity;
            }

            telemetry.addData("current velocity", currentVelocity);
            telemetry.addData("maximum velocity", maxVelocity);
            telemetry.update();
        }
    }
}
