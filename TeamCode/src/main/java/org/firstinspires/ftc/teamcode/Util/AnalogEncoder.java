//package org.firstinspires.ftc.teamcode.Util;
//
//import com.qualcomm.robotcore.hardware.AnalogInput;
//
//public class AnalogEncoder {
//    private AnalogInput analogInput;
//    private double voltageRange;
//    private int ticksPerRevolution;
//
//    public AnalogEncoder(AnalogInput analogInput, double voltageRange, int ticksPerRevolution) {
//        this.analogInput = analogInput;
//        this.voltageRange = voltageRange; // e.g., 3.3V or 5.0V depending on encoder
//        this.ticksPerRevolution = ticksPerRevolution; // number of ticks in one full rotation
//    }
//
//    public double getVoltage() {
//        return analogInput.getVoltage();
//    }
//
//    public double getPositionInTicks() {
//        double voltage = getVoltage();
//        return (voltage / voltageRange) * ticksPerRevolution;
//    }
//
//    public double getRotation() {
//        // Returns a value from 0.0 to 1.0 representing one full revolution
//        return getVoltage() / voltageRange;
//    }
//
//    public double getDegrees() {
//        return getRotation() * 360.0;
//    }
//}

package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class AnalogEncoder {
    private AnalogInput analogInput;
    private double voltageRange;
    private int ticksPerRevolution;
    private double gearRatio; // encoder rotations per mechanism rotation

    /**
     * @param analogInput the analog input from the encoder
     * @param voltageRange the encoder's voltage range (e.g. 3.3V or 5.0V)
     * @param ticksPerRevolution the number of ticks in one encoder revolution
     * @param gearRatio the gear ratio between the encoder and the mechanism
     *                  (encoder rotations per mechanism rotation)
     *                  Example: if encoder spins twice for every 1 mechanism rotation, gearRatio = 2.0
     */
    public AnalogEncoder(AnalogInput analogInput, double voltageRange, int ticksPerRevolution, double gearRatio) {
        this.analogInput = analogInput;
        this.voltageRange = voltageRange;
        this.ticksPerRevolution = ticksPerRevolution;
        this.gearRatio = gearRatio;
    }

    public double getVoltage() {
        return analogInput.getVoltage();
    }

    public double getPositionInTicks() {
        double voltage = getVoltage();
        return (voltage / voltageRange) * ticksPerRevolution / gearRatio;
    }

    public double getRotation() {
        // Returns a value from 0.0 to 1.0 representing one full revolution of the mechanism
        return (getVoltage() / voltageRange) / gearRatio;
    }

    public double getDegrees() {
        // Returns the rotation in degrees, accounting for gear ratio
        return getRotation() * 360.0;
    }

    public double getGearRatio() {
        return gearRatio;
    }

    public void setGearRatio(double gearRatio) {
        this.gearRatio = gearRatio;
    }
}
