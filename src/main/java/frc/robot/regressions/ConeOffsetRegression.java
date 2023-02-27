// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.regressions;

import frc.lib.team254.util.PolynomialRegression;

/** Add your docs here. */
public class ConeOffsetRegression {
    private static double[][] midPracticeField = { 
    };
    private static double[][] highPracticeField  = {
        { 0, 0}
    };
    public static PolynomialRegression midPracticeFieldReg = new PolynomialRegression(midPracticeField, 1);

    public static PolynomialRegression highPracticeFieldReg = new PolynomialRegression(highPracticeField, 1);




}
