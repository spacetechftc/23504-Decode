/*package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

public class ATurret implements Subsystem {

    DcMotorEx turret;
    public static int powerPosition = 0;


    public static int pos = 0;
    public static double blueGoalX = 0;
    public static double blueGoalY = 144;
    public static double redGoalX  = 144;
    public static double redGoalY  = 144;

    public void setTurretPosition(int pos) {
        turret.setPositionPIDFCoefficients(17);
        turret.setTargetPosition(powerPosition);
        turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        turret.setPower(1);
    }

    public void resetEncoder() {
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }

    public void alignTurret(double x, double y, double heading, boolean blue) {
        double headingDeg = Math.toDegrees(heading);

        double goalX = blue ? blueGoalX : redGoalX;
        double goalY = blue ? blueGoalY : redGoalY;

        double angleToGoal = Math.toDegrees(Math.atan2(goalX - x, goalY - y));

        double turretAngle = angleToGoal + headingDeg - 90;

       // int targetTicks = (int) (tSlope * turretAngle);

        final int TURRET_MIN = -1000;
        final int TURRET_MAX = 1000;

        if (targetTicks > TURRET_MAX || targetTicks < TURRET_MIN) {
            pos = 0;
        } else {
            pos = targetTicks;
        }

        double distance = Math.hypot(goalX - x, goalY - y);
        distance = (int) distance;
       // vel = (int) (distance * fSlope + fIntercept);

        setTurretPosition(pos);
    }

    @Override
    public void initialize() {
        turret = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "turret_motor");
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}

 */
