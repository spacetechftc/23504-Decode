package org.firstinspires.ftc.teamcode.opModes.tests;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "TuneKs", group = "OpModes Tests")
@Configurable
public class TuneKs extends LinearOpMode {

    public static double power = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor;
        DcMotorEx turretEncoder = hardwareMap.get(DcMotorEx.class, "turret_motor");

        // Resetar para começar do zero
        turretEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor = hardwareMap.get(DcMotorEx.class, "turret_motor");

        telemetry.addLine("Ready. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            motor.setPower(power);
            int position = turretEncoder.getCurrentPosition();
            telemetry.addData("Encoder Position", position);
            telemetry.update();
        }
    }
}