// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public class ElevatorWristState {
    public final Rotation2d angle;
    public final double height;
    
    public ElevatorWristState(Rotation2d angle, double height){
        this.angle = angle;
        this.height = height;
    }
    public ElevatorWristState(double angle, double height){
        this(Rotation2d.fromDegrees(angle), height);
    }
    
}
