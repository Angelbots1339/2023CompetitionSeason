// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.regressions;

import frc.lib.team254.util.PolynomialRegression;

/** Add your docs here. */
public class CubeShootRegression {
    private static final double[][] angle = {
        {1, 10}//at distance of 1 meter angle intake at 10 degrees 
    };

    private static final double[][] PercentOutput = {
        {1, 0.2}//at distance of 1 meter angle intake at 0.2 percent 
    };

    public static final PolynomialRegression angleRegression = new PolynomialRegression(angle, 1);
    public static final PolynomialRegression percentRegression = new PolynomialRegression(PercentOutput, 1);
}
