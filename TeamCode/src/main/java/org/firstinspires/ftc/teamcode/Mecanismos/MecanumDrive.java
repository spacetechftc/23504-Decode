package org.firstinspires.ftc.teamcode.Mecanismos;

import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.extensions.pedro.PedroDriverControlled;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.hardware.driving.DriverControlledCommand;
import dev.nextftc.hardware.impl.MotorEx;

public class MecanumDrive {
    private boolean autoAlignEnabled = false;
    private double autoHeading = 0;
    public void setAutoAlign(boolean enabled, double strafe) {
        this.autoAlignEnabled = enabled;
        this.autoHeading = strafe;
    }
    private final MotorEx frontLeftMotor = new MotorEx("front_left").reversed().brakeMode(); // Port 3
    private final MotorEx frontRightMotor = new MotorEx("front_right").brakeMode(); // Port 0

    private final MotorEx backLeftMotor = new MotorEx("back_left").brakeMode(); // Port 2
    private final MotorEx backRightMotor = new MotorEx("back_right").reversed().brakeMode(); // Port 1

    public void
    start() {
        DriverControlledCommand driverControlled = new PedroDriverControlled(
                () -> (double) -ActiveOpMode.gamepad1().left_stick_y, // Forward e Backward
                () -> (double) -ActiveOpMode.gamepad1().left_stick_x, // Strafe
                () -> autoAlignEnabled ? autoHeading :(double) -ActiveOpMode.gamepad1().right_stick_x, // Turn
                true
        );
        driverControlled.schedule();
    }

    public void stop() {
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
    }

    public void driveRobotCentric(double forward, double strafe, double turn) {
        double fl = forward + strafe + turn;
        double bl = forward - strafe + turn;
        double fr = forward - strafe - turn;
        double br = forward + strafe - turn;

        double max = Math.max(1.0, Math.max(
                Math.abs(fl),
                Math.max(Math.abs(bl), Math.max(Math.abs(fr), Math.abs(br)))
        ));

        frontLeftMotor.setPower(fl / max);
        backLeftMotor.setPower(bl / max);
        frontRightMotor.setPower(fr / max);
        backRightMotor.setPower(br / max);
    }

    public double getAutoStrafeFromLimelight() {
        double tx = LimelightSubsystem.INSTANCE.tx;
        double strafe = tx * 0.02;

        if (Math.abs(tx) < 1.0) {
            return 0;
        }

        strafe = Math.max(-0.35, Math.min(0.35, strafe));
        return strafe;
    }
    public Command alignToBalls = new LambdaCommand()
            .setUpdate(() -> {
                    if (LimelightSubsystem.INSTANCE.hasTarget()) {
                        double strafe = getAutoStrafeFromLimelight();
                        driveRobotCentric(0, strafe, 0);
                    } else {
                        stop();
                    }
            })
            .setIsDone(() -> Math.abs(LimelightSubsystem.INSTANCE.tx) < 2);


}
