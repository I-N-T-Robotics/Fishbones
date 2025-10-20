package org.firstinspires.ftc.teamcode.Util;

public class Filters {

    public static double applyDeadband(double value, double deadband) {
        return (Math.abs(value) > deadband ? value : 0);
    }

    public static double smoothInput(double currentValue, double previousValue, double alpha) {
        return alpha * currentValue + (1.0 - alpha) * previousValue;
    }
}