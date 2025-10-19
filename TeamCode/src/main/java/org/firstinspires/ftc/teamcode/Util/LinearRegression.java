package org.firstinspires.ftc.teamcode.Util;

import edu.wpi.first.math.geometry.Translation2d;

public class LinearRegression {

    private final Translation2d[] points;
    private final Translation2d coefficient;

    public LinearRegression(Translation2d[] points) {
        this.points = points;
        this.coefficient = estimateSlope();
    }

    private Translation2d estimateSlope() {
        Translation2d mean = getMean(points);

        double xy = 0;
        double xx = 0;

        for (Translation2d point : points) {
            double x = point.getX();
            double y = point.getY();

            xy += (x - mean.getX()) * (y - mean.getY());
            xx += (x - mean.getX()) * (x - mean.getX());
        }

        double slope = xy / xx;
        double yint = mean.getY() - slope * mean.getX();

        return new Translation2d(slope, yint);
    }

    private Translation2d getMean(Translation2d[] points) {
        double sumX = 0;
        double sumY = 0;

        for (Translation2d point : points) {
            sumX += point.getX();
            sumY += point.getY();
        }

        return new Translation2d(sumX / points.length, sumY / points.length);
    }

    private double getSlope() {
        return coefficient.getX();
    }

    private double getYIntercept() {
        return coefficient.getY();
    }

    public double calculatePoint(double x) {
        return getSlope() * x + getYIntercept();
    }
}