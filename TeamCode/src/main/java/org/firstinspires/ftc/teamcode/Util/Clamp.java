package org.firstinspires.ftc.teamcode.Util;

public final class Clamp {
    public static double clamp(double x, double min, double max) {
        if (min < max) {
            if (x > max) return max;
            if (x < min) return min;
        } else {
            if (x > min) return min;
            if (x < max) return max;
        }
        return x;
    }

    public static double clamp(double x, double max) {
        return clamp(x, -max, max);
    }
}
