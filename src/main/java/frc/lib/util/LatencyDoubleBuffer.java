// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.wpilibj.Timer;

/** Add your docs here. */
public class LatencyDoubleBuffer {
    // Declaring the class variables.
    public volatile int count, front, rear;
    private final double period;
    public volatile double[] queue;

    public LatencyDoubleBuffer(int capacity, double period) {
        queue = new double[capacity];
        front = 0;
        rear = 0;
        count = 0;
        this.period = period;
    }
    public void addMeasurement(Double item) {
        queue[rear] = item;
        rear = (rear + 1) % queue.length;
        if (count < queue.length) {
            count++;
            return;
        }
        front = rear;
    }
    public double getMeasurement(double latency){
        if(latency < 0) latency = 0;
        if(latency > (queue.length - 1) * period) latency = (queue.length - 1) * period;
        int i  = (front + (int)Math.round(latency / period)) % queue.length;
        return queue[i];
    }

    public double getMeasurementAtSeconds(Double timeInSec){
        return getMeasurement((Timer.getFPGATimestamp() - timeInSec) * 1000);
    }
}
