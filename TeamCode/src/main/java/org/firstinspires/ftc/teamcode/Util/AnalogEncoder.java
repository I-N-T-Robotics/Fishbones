//package org.firstinspires.ftc.teamcode.Util;
//
//import com.qualcomm.robotcore.hardware.AnalogInput;
//
//public class AnalogEncoder {
//    private AnalogInput analogInput;
//    private double voltageRange;
//    private int ticksPerRevolution;
//    private double gearRatio; // encoder rotations per mechanism rotation
//
//    /**
//     * @param analogInput the analog input from the encoder
//     * @param voltageRange the encoder's voltage range (e.g. 3.3V or 5.0V)
//     * @param ticksPerRevolution the number of ticks in one encoder revolution
//     * @param gearRatio the gear ratio between the encoder and the mechanism
//     *                  (encoder rotations per mechanism rotation)
//     *                  Example: if encoder spins twice for every 1 mechanism rotation, gearRatio = 2.0
//     */
//    public AnalogEncoder(AnalogInput analogInput, double voltageRange, int ticksPerRevolution, double gearRatio) {
//        this.analogInput = analogInput;
//        this.voltageRange = voltageRange;
//        this.ticksPerRevolution = ticksPerRevolution;
//        this.gearRatio = gearRatio;
//    }
//
//    public double getVoltage() {
//        return analogInput.getVoltage();
//    }
//
//    public double getPositionInTicks() {
//        double voltage = getVoltage();
//        return (voltage / voltageRange) * ticksPerRevolution / gearRatio;
//    }
//
//    public double getRotation() {
//        // Returns a value from 0.0 to 1.0 representing one full revolution of the mechanism
//        return (getVoltage() / voltageRange) / gearRatio;
//    }
//
//    public double getDegrees() {
//        // Returns the rotation in degrees, accounting for gear ratio
//        return getRotation() * 360.0;
//    }
//
//    public double getGearRatio() {
//        return gearRatio;
//    }
//
//    public void setGearRatio(double gearRatio) {
//        this.gearRatio = gearRatio;
//    }
//}

package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class AnalogEncoder {
    private final AnalogInput analogInput;
    private final double voltageRange;
    private final double gearRatio; // encoder rotations per mechanism rotation

    /**
     * @param analogInput the analog input from the encoder
     * @param voltageRange the encoder's voltage range (e.g. 3.3V or 5.0V)
     * @param gearRatio encoder rotations per mechanism rotation (e.g. 2.0 means encoder spins twice per one module rotation)
     */
    public AnalogEncoder(AnalogInput analogInput, double voltageRange, double gearRatio) {
        this.analogInput = analogInput;
        this.voltageRange = voltageRange;
        this.gearRatio = gearRatio;
    }

    /** Raw analog voltage (0–voltageRange) */
    public double getVoltage() {
        return analogInput.getVoltage();
    }

    /** Returns the mechanism’s current rotation in degrees (0–360) */
    public double getDegrees() {
        // Map voltage proportionally to encoder angle (0–360)
        double encoderDegrees = (getVoltage() / voltageRange) * 360.0;
        // Adjust for gear ratio (encoder turns multiple times per mechanism revolution)
        double mechanismDegrees = encoderDegrees / gearRatio;
        // Normalize to 0–360 range
        mechanismDegrees = ((mechanismDegrees % 360) + 360) % 360;
        return mechanismDegrees;
    }

    /** Returns rotation in 0.0–1.0 range (fraction of one full mechanism revolution) */
    public double getRotation() {
        return getDegrees() / 360.0;
    }
}