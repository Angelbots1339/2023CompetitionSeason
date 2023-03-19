// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.regressions;

import frc.lib.team254.util.PolynomialRegression;

/** Add your docs here. */
public class ConeOffsetRegression {
    private static double[][] midPracticeField = {
        { 0, 0},
        { -0.030000, 4.8},
        {0.031000, 0.86}
    };
    private static double[][] highPracticeField  = {
        { 0, 0},
        {-0.030000, 2.08},
        {0.031000, -0.80}
    };
    public static PolynomialRegression midPracticeFieldReg = new PolynomialRegression(midPracticeField, 1);

    
    public static PolynomialRegression highPracticeFieldReg = new PolynomialRegression(highPracticeField, 1);


    private static double[][] midCompField = {
        { 0.07, 0},
        { 0.044000, 2.29},
        {0.155000, 1.10}
    };
    private static double[][] highCompField  = {
        { 0.07, 0},
        { 0.044000, 1.70},
        {0.155000, -2.60}
    };
    public static PolynomialRegression midCompFieldReg = new PolynomialRegression(midCompField, 1);

    public static PolynomialRegression highCompFieldReg = new PolynomialRegression(highCompField, 1);



}
