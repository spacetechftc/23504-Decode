package org.firstinspires.ftc.teamcode.Mecanismos;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.Subsystems.LimelightSubsystem;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.extensions.pedro.PedroComponent;
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

    public void start() {
        DriverControlledCommand driverControlled = new PedroDriverControlled(
                () -> (double) -ActiveOpMode.gamepad1().left_stick_y, // Forward e Backward
                () -> (double) -ActiveOpMode.gamepad1().left_stick_x, // Strafe
                () -> (double) -ActiveOpMode.gamepad1().right_stick_x, // Turn
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
        double strafe = -tx * 1;

        strafe = Math.max(-0.45, Math.min(0.45, strafe));
        return strafe;
    }
    public Command alignToBallsAuto() {
        return new LambdaCommand()
                .setStart(() -> {
                    LimelightSubsystem.INSTANCE.artifact.invoke();
                })
                .setUpdate(() -> {
                    if (LimelightSubsystem.INSTANCE.hasTarget()) {
                        double strafe = getAutoStrafeFromLimelight();
                        driveRobotCentric(0, strafe, 0);
                    } else {
                        stop();
                    }
                })
                /*.setIsDone(() ->
                      //  LimelightSubsystem.INSTANCE.hasTarget() &&
                             //   Math.abs(LimelightSubsystem.INSTANCE.tx) < 2
                )

                 */
                .setStop(interrupted -> stop());
    }

    public Command alignToBallsWithPedro() {
        return new LambdaCommand()
                .setStart(() -> {
                    LimelightSubsystem.INSTANCE.artifact.invoke();
                })
                .setUpdate(() -> {
                    // só gera correção quando o follower estiver livre
                    if (PedroComponent.follower().isBusy()) return;
                    if (!LimelightSubsystem.INSTANCE.hasTarget()) return;

                    double tx = LimelightSubsystem.INSTANCE.tx;

                    // tolerância
                    if (Math.abs(tx) < 2.0) return;

                    Pose current = PedroComponent.follower().getPose();

                    // ganho câmera -> deslocamento lateral
                    double lateralOffset = tx * 0.20;

                    // clamp para não exagerar
                    lateralOffset = Math.max(-3.0, Math.min(3.0, lateralOffset));

                    double heading = current.getHeading();

                    // converte deslocamento lateral do robô para coordenadas do campo
                    double dx = -Math.sin(heading) * lateralOffset;
                    double dy =  Math.cos(heading) * lateralOffset;

                    Pose alignedPose = new Pose(
                            current.getX() + dx,
                            current.getY() + dy,
                            current.getHeading()
                    );

                    PathChain correction = PedroComponent.follower().pathBuilder()
                            .addPath(new BezierLine(current, alignedPose))
                            .setLinearHeadingInterpolation(current.getHeading(), current.getHeading())
                            .build();

                    PedroComponent.follower().followPath(correction, true);
                })
                .setIsDone(() ->
                        LimelightSubsystem.INSTANCE.hasTarget() &&
                                Math.abs(LimelightSubsystem.INSTANCE.tx) < 2.0 &&
                                !PedroComponent.follower().isBusy()
                );
    }
}
