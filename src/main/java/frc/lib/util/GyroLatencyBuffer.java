// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import java.util.Timer;

import com.ctre.phoenix.sensors.Pigeon2;

/** Add your docs here. */
public class GyroLatencyBuffer {
    Timer timer = new Timer();
    LatencyDoubleBuffer yaw;
    LatencyDoubleBuffer pitch;
    LatencyDoubleBuffer roll;
    Pigeon2 gyro;
    long period;

    public GyroLatencyBuffer(Pigeon2 gyro, int capacity, long period){
        yaw = new LatencyDoubleBuffer(capacity, period);
        pitch = new LatencyDoubleBuffer(capacity, period);
        roll = new LatencyDoubleBuffer(capacity, period);
        this.gyro = gyro;
        this.period = period;
    };

    public void start(){
        timer.scheduleAtFixedRate(new java.util.TimerTask(){
            @Override
            public void run() {
                
                yaw.addMeasurement(gyro.getYaw());
                pitch.addMeasurement(gyro.getPitch());
                roll.addMeasurement(gyro.getRoll());
            }
        }, 0, period);
    }

    public void stop(){
        timer.cancel();
    }

    public double getYaw(double latency){
        return yaw.getMeasurement(latency);
    }

    public double getYawAtSeconds(double timeInSec){
        return yaw.getMeasurementAtSeconds(timeInSec);
    }

    public double getPitch(double latency){
        return pitch.getMeasurement(latency);
    }

    public double getPitchAtSeconds(double timeInSec){
        return pitch.getMeasurementAtSeconds(timeInSec);
    }


    public double getRoll(double latency){
        return roll.getMeasurement(latency);
    }

    public double getRollAtSeconds(double timeInSec){
        return roll.getMeasurementAtSeconds(timeInSec);
    }

}
