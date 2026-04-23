package org.firstinspires.ftc.teamcode.Mecanismos;

public class PIDFController {
    // PIDF coefficients
    public static double kP, kI, kD, kF;

    // State variables
    private double sumError = 0;
    private double lastError = 0;

    public PIDFController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public void reset() {
        sumError = 0;
        lastError = 0;
    }

    public double calculate(double target, double current) {
        double error = target - current;

        // Proportional
        double P = kP * error;

        sumError += error;

        double I = kI * sumError;

        // Derivative
        double derivative = lastError - error;
        double D = kD * derivative;

        // Feedforward
        double F = kF * target;

        // Store for next loop
        lastError = error;

        // Total output
        return P + I + D + F;
    }
}