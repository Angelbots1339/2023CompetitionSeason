// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.team254.util;

import edu.wpi.first.math.system.plant.DCMotor;

/** Add your docs here. */
public class SimulationUtils {

    public static DCMotor falconSim(int NumOfMotors){
        return new DCMotor(
            12,
         4.69,
          257,
           1.5,
           6380/60 * 2 * Math.PI,
             NumOfMotors);

    }
}
