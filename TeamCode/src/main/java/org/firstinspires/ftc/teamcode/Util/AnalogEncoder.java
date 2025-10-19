package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class AnalogEncoder {
    private AnalogInput analogInput;
    private double voltageRange;
    private int ticksPerRevolution;

    public AnalogEncoder(AnalogInput analogInput, double voltageRange, int ticksPerRevolution) {
        this.analogInput = analogInput;
        this.voltageRange = voltageRange; // e.g., 3.3V or 5.0V depending on encoder
        this.ticksPerRevolution = ticksPerRevolution; // number of ticks in one full rotation
    }

    public double getVoltage() {
        return analogInput.getVoltage();
    }

    public double getPositionInTicks() {
        double voltage = getVoltage();
        return (voltage / voltageRange) * ticksPerRevolution;
    }

    public double getRotation() {
        // Returns a value from 0.0 to 1.0 representing one full revolution
        return getVoltage() / voltageRange;
    }

    public double getDegrees() {
        return getRotation() * 360.0;
    }
}