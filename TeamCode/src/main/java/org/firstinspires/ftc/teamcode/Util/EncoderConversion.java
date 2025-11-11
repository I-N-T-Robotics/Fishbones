package org.firstinspires.ftc.teamcode.Util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class EncoderConversion {
    private final DcMotorEx encoder;
    private final double positionConversionFactor;
    private final double velocityConversionFactor;
    private double lastPosition = 0;
    private double lastTimeStamp = 0;
    private ElapsedTime timer = new ElapsedTime();

    public EncoderConversion(DcMotorEx encoder, double wheelDiameterMeters, double gearRatio, int countsPerRevolution) {
        this.encoder = encoder;

        double wheelCircumference = Math.PI * wheelDiameterMeters;

        this.positionConversionFactor = wheelCircumference / (gearRatio * countsPerRevolution);
        this.velocityConversionFactor = positionConversionFactor;

        lastTimeStamp = timer.seconds();
        lastPosition = encoder.getCurrentPosition();
    }

    public double getPositionMeters() {
        return encoder.getCurrentPosition() * positionConversionFactor;
    }

    public double getVelocityMetersPerSecond() {
        double currentTime = timer.seconds();
        double currentPosition = encoder.getCurrentPosition();

        double deltaPosition = currentPosition - lastPosition;
        double deltaTime = currentTime - lastTimeStamp;

        double velocity = 0;
        if(deltaTime > 0) {
            velocity = -(deltaPosition / deltaTime) * velocityConversionFactor;
        }

        lastTimeStamp = currentTime;
        lastPosition = currentPosition;

        return velocity;
    }

    public void reset() {
        encoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lastPosition = 0;
        lastTimeStamp = timer.seconds();
        encoder.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}