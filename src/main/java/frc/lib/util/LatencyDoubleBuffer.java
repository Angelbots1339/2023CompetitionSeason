// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

/** Add your docs here. */
public class LatencyDoubleBuffer {
    // Declaring the class variables.
    private int count, front, rear;
    private double period;
    private double[] queue;

    public LatencyDoubleBuffer(int capacity, double period) {
        queue = new double[capacity];
        front = 0;
        rear = 0;
        count = 0;
        this.period = period;
    }

    public void addMeasurement(Double item) {
        if (count == queue.length) {
            Double i = queue[front];
            front = (front + 1) % queue.length;
            count--;
            queue[rear] = item;
            rear = (rear + 1) % queue.length;

        } else {
            queue[rear] = item;
            rear = (rear + 1) % queue.length;
        }
        count++;
    }

    public boolean isEmpty() {
        return count == 0;
    }

    public double getMeasurement(double Latency){
       int i  = (rear + (int)Math.round(Latency / period)) % queue.length;
       return queue[i];
    }

}
