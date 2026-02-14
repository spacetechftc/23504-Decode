package org.firstinspires.ftc.teamcode.opModes.tests;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "Turret Encoder Test", group = "OpModes Tests")
public class TurretEncoderTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx turretEncoder = hardwareMap.get(DcMotorEx.class, "turret_motor");

        // Resetar para começar do zero
        turretEncoder.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretEncoder.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        telemetry.addLine("Ready. Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            int position = turretEncoder.getCurrentPosition();
            telemetry.addData("Encoder Position", position);
            telemetry.update();
        }
    }
}