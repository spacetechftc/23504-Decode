package org.firstinspires.ftc.teamcode.Subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Mecanismos.PIDFController;

import dev.nextftc.core.subsystems.Subsystem;
import dev.nextftc.ftc.ActiveOpMode;

@Configurable
public class testTurret implements Subsystem {

    DcMotorEx turretMotor;
    public int currentTicks, targetTicks;
    public double distance, movedDistance, flightTime;

    PIDFController odometryTicksControl;
    PIDFController limelightControl;

    // Configuração do PID -- Odometria
    public static double KP = 0.00084;
    public static double KD = 0.000015;

    // Ângulo da Torreta
    public static double GEAR_RATIO = 106.0/37.0; // Your gear ratio
    private static final double ENCODER_CPR = 8192;
    private static final double TICKS_PER_TURRET_REV = ENCODER_CPR * GEAR_RATIO;

    // GOALS
    public static double blueGoalX = 6;
    public static double blueGoalY = 120;
    public static double redGoalX  = 128;
    public static double redGoalY  = 120;
    public double power = 0;
    // Instância da Torreta
    public static final testTurret INSTANCE = new testTurret();
    private testTurret() {}

    // Controle baseado na odometria
    private int degreesToTicks(double degrees) {
        return (int) Math.round((degrees / 360.0) * TICKS_PER_TURRET_REV);
    }
    private double ticksToDegrees(int ticks) {
        return (ticks / TICKS_PER_TURRET_REV) * 360.0;
    }

    private static final double[] DISTANCES = {20, 30, 40, 50, 60, 70, 80, 90, 100};
    private static final double[] FLIGHT_TIMES = {0.6, 0.6, 0.6, 0.61, 0.61, 0.62, 0.62, 0.62, 0.625};

    private double flightTime(double distance) {
        if (distance <= DISTANCES[0]) return FLIGHT_TIMES[0];
        if (distance >= DISTANCES[DISTANCES.length - 1]) return FLIGHT_TIMES[FLIGHT_TIMES.length - 1];

        for (int i = 0; i < DISTANCES.length - 1; i++) {
            double d1 = DISTANCES[i];
            double d2 = DISTANCES[i + 1];

            if (distance >= d1 && distance <= d2) {
                double t1 = FLIGHT_TIMES[i];
                double t2 = FLIGHT_TIMES[i + 1];

                double alpha = (distance - d1) / (d2 - d1);
                return t1 + alpha * (t2 - t1);
            }
        }

        return FLIGHT_TIMES[FLIGHT_TIMES.length - 1];
    }

    public void resetEncoder() {
        turretMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
    }

    public double alignTurretWithMotion(double x, double y, double heading, double vx, double vy, int currentTicks, boolean blue) {
        double headingDeg = Math.toDegrees(heading);

        double goalX = blue ? blueGoalX : redGoalX;
        double goalY = blue ? blueGoalY : redGoalY;

        distance = Math.hypot(goalX - x, goalY - y);
        flightTime = 0.6;

        double movedGoalX = goalX - vx * flightTime;
        double movedGoalY = goalY - vy * flightTime;

        movedDistance = Math.hypot(movedGoalX - x, movedGoalY - y);

        double angleToGoal = Math.toDegrees(Math.atan2(movedGoalY - y, movedGoalX - x));

        double turretAngle = normalizeAngle(headingDeg - angleToGoal);

        targetTicks = degreesToTicks(turretAngle);
        power = odometryTicksControl.calculate(targetTicks, currentTicks);
        power = Math.max(-1, Math.min(1, power));

        return power;
    }

    public double alignTurretWithOdometry(double x, double y, double heading, int currentTicks, boolean blue) {
        double headingDeg = Math.toDegrees(heading);

        double goalX = blue ? blueGoalX : redGoalX;
        double goalY = blue ? blueGoalY : redGoalY;
        distance = Math.hypot(goalX - x, goalY - y);

        double angleToGoal = Math.toDegrees(Math.atan2(goalY - y, goalX - x));

        double turretAngle = normalizeAngle(headingDeg - angleToGoal);

        targetTicks = degreesToTicks(turretAngle);
        power = odometryTicksControl.calculate(targetTicks, currentTicks);
        power = Math.max(-1, Math.min(1, power));

        return power;
    }


    public void alignTurretTeleOp(double x, double y, double heading, double vx, double vy, int currentTicks, boolean blue) {
        power = alignTurretWithMotion(x,y,heading,vx,vy,currentTicks,blue);

        turretMotor.setPower(power);
    }

    public void alignTurretAuto(double x, double y, double heading, int currentTicks, boolean blue) {
        power = alignTurretWithOdometry(x, y, heading, currentTicks, blue);

        turretMotor.setPower(power);
    }


    /*
    public void alignTurret(double x, double y, double heading, int currentTicks, boolean blue) {
        power = alignTurretWithOdometry(x,y,heading,currentTicks, false);

        turretMotor.setPower(power);
    }

     */

    public void manualTurret(boolean left, boolean right) {
        if (left) {
            turretMotor.setPower(-1);
        } else if (right) {
            turretMotor.setPower(1);
        } else {
            turretMotor.setPower(0);
        }
    }

    public void turretToPosition(int pos) {
        targetTicks = pos;
        power = odometryTicksControl.calculate(targetTicks, currentTicks);
        power = Math.max(-1, Math.min(1, power));
        turretMotor.setPower(power);
    }

    private double normalizeAngle(double angle) {
        angle = angle % 360;
        if (angle > 180) angle -= 360;
        if (angle < -180) angle += 360;
        return angle;
    }

    @Override
    public void initialize() {
        turretMotor = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "turret_motor");
        turretMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        odometryTicksControl = new PIDFController(KP, 0, KD, 0);
    }

    @Override
    public void periodic() {
        currentTicks = turretMotor.getCurrentPosition();
        ActiveOpMode.telemetry().addData("Turret", "------------------");
        ActiveOpMode.telemetry().addData("CurrentTicks", currentTicks);
        ActiveOpMode.telemetry().addData("TargetTicks", targetTicks);
        ActiveOpMode.telemetry().addData("Degrees", ticksToDegrees(currentTicks));
        ActiveOpMode.telemetry().addData("Distance", movedDistance);

    }
}