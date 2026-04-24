package org.firstinspires.ftc.teamcode.Mecanismos;

public class PIDFController {
    // PIDF coefficients
    public double kP, kI, kD, kF;

    // State variables
    private double sumError = 0;
    private double lastError = 0;
    private double lastTime = 0;

    public PIDFController(double kP, double kI, double kD, double kF) {
        setPIDF(kP, kI, kD, kF);
    }

    public void setPIDF(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }
    public void reset() {
        sumError = 0;
        lastError = 0;
        lastTime = 0;
    }

    public double calculate(double target, double current) {
        double currentTime = System.nanoTime() / 1e9;

        double dt = 0;
        if (lastTime != 0) {
            dt = currentTime - lastTime;
        }

        double error = target - current;

        // P
        double p = kP * error;

        // I (com dt)
        if (dt > 0) {
            sumError += error * dt;
        }
        double i = kI * sumError;

        // D (com dt)
        double d = 0;
        if (dt > 0) {
            d = kD * ((error - lastError) / dt);
        }

        // F
        double f = kF * target;

        lastError = error;
        lastTime = currentTime;

        return p + i + d + f;
    }
}