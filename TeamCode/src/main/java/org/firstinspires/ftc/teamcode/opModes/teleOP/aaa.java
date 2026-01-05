/*package org.firstinspires.ftc.teamcode.Subsystems;

import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.globals.StringNames;
import org.firstinspires.ftc.teamcode.globals.Values;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.Gamepads;
import dev.nextftc.hardware.driving.MecanumDriverControlled;
import dev.nextftc.hardware.impl.MotorEx;

public class aaa implements Subsystem {

    public static final aaa INSTANCE = new aaa();
    private aaa() {}

    public StringNames stringNames = new StringNames();
    public Values values = new Values();
    public final MotorEx frontLeftMotor = new MotorEx(stringNames.frontLeft_motor_name).reversed();
    public final MotorEx frontRightMotor = new MotorEx(stringNames.backLeft_motor_name).reversed();
    public final MotorEx backLeftMotor = new MotorEx(stringNames.backRight_motor_name);
    public final MotorEx backRightMotor = new MotorEx(stringNames.frontRight_motor_name);
    public MecanumDriverControlled mecanumDriverController;
    private GoBildaPinpointDriver pinpoint;
    private PIDController headingPID;

    private boolean headingLock = false;
    private double targetHeading = 0.0; // radiani
    private double headingCorrection = 0;

    @Override
    public void initialize() {
        HardwareMap hw = ActiveOpMode.hardwareMap();

        headingPID = new PIDController(1,0,0);
    }

    public void setHeadingLock(boolean enabled) {
        headingLock = enabled;
        if (enabled) {
            targetHeading = getCurrentHeading();
        }
    }

    public boolean isHeadingLockEnabled() {
        return headingLock;
    }

    public void drive(double forward, double strafe, double turnInput) {
        headingCorrection = turnInput;

        if (headingLock) {
            double currentHeading = getCurrentHeading();
            double headingError = targetHeading - currentHeading;
            headingError = Math.IEEEremainder(headingError, 2 * Math.PI);

            if (Math.abs(headingError) < Math.toRadians(2)) {
                headingCorrection = 0;
            } else {
                headingCorrection = headingPID.calculate(headingError);
            }

            mecanumDriverController = new MecanumDriverControlled(
                    frontLeftMotor,
                    frontRightMotor,
                    backLeftMotor,
                    backRightMotor,
                    Gamepads.gamepad1().leftStickY(),
                    Gamepads.gamepad1().leftStickX(),
                    () -> headingCorrection
            );
        }
        else{
            mecanumDriverController = new MecanumDriverControlled(
                    frontLeftMotor,
                    frontRightMotor,
                    backLeftMotor,
                    backRightMotor,
                    Gamepads.gamepad1().leftStickY(),
                    Gamepads.gamepad1().leftStickX(),
                    Gamepads.gamepad1().rightStickX()
            );
        }


    }
}

 */



