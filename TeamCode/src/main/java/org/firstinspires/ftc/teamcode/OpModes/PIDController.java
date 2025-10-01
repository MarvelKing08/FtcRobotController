package org.firstinspires.ftc.teamcode.OpModes;

public class PIDController {
    private double kP, kI, kD;
    private double integral, previousError;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.integral = 0;
        this.previousError = 0;
    }

    public double calculate(double target, double current) {
        double error = target - current; // in your case, target is 0
        integral += error;
        double derivative = error - previousError;
        previousError = error;

        double output = (kP * error) + (kI * integral) + (kD * derivative);

        // Clamp the output to motor range
        if (output > .8) output = .8;
        if (output < -.8) output = -.8;

        return output;
    }
}
