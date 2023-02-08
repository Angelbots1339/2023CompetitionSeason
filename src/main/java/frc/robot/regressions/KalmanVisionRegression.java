// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.regressions;

import frc.lib.team254.util.PolynomialRegression;

/** Add your docs here. */
public class KalmanVisionRegression {

    // TODO calculate these
    /*
     * Standard deviations of the vision measurements. Increase these
     * numbers to trust global measurements from vision less. This matrix is in the
     * form [x, y]ᵀ,
     * with units in meters and radians.
     */
    private static double[][] xyStdDev = { //Values from 6238 see https://github.com/Mechanical-Advantage/RobotCode2023/blob/main/src/main/java/org/littletonrobotics/frc2023/subsystems/apriltagvision/AprilTagVision.java
            { 0.752358, 0.005 },
            { 1.016358, 0.0135 },
            { 1.296358, 0.016 },
            { 1.574358, 0.038 },
            { 1.913358, 0.0515 },
            { 2.184358, 0.0925 },
            { 2.493358, 0.0695 },
            { 2.758358, 0.046 },
            { 3.223358, 0.1245 },
            { 4.093358, 0.0815 },
            { 4.726358, 0.193 }
    };
  

    public static PolynomialRegression xyStdDevReg = new PolynomialRegression(xyStdDev, 1);

}
