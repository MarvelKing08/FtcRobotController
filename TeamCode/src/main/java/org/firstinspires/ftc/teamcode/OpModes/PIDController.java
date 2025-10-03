package org.firstinspires.ftc.teamcode.OpModes;

public class PIDController {
    private static double kP, kI, kD;
    public static double integral, previousError;



    public static double calculate(double target, double current) {
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
    public double pid(double target, double current) {
        return target + current;
    }
}

