// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.regressions;

import frc.lib.team254.util.PolynomialRegression;

/** Add your docs here. */
public class ConeOffsetRegression {
    private static double[][] midPracticeField = {
        { 0, 0},
        { 0.155000, 2.01},
        {-0.184000, -6.94}
    };
    private static double[][] highPracticeField  = {
        { 0, 0},
        { 0.155000, 2.4},
        {-0.184000, -3.65}
    };
    public static PolynomialRegression midPracticeFieldReg = new PolynomialRegression(midPracticeField, 1);

    public static PolynomialRegression highPracticeFieldReg = new PolynomialRegression(highPracticeField, 1);




}
