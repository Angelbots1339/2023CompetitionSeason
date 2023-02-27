// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.lib.team254.util.PolynomialRegression;
import frc.lib.util.ElevatorWristState;
import frc.robot.regressions.ConeOffsetRegression;

/** Add your docs here. */
public class FieldDependentConstants {
    
    public static class FieldConstants {
        
        //Defualt values
        public static PolynomialRegression MID_CONE_Regression = ConeOffsetRegression.midPracticeFieldReg;
        public PolynomialRegression HIGH_CONE_Regression = ConeOffsetRegression.highPracticeFieldReg;
        
        //intake positions
        public ElevatorWristState HIGH_CONE = new ElevatorWristState(105, 1.31);
        public double HIGH_CONE_HIGHT_BEFORE_ANGLE = 0.1;
        public double HIGH_CONE_OUTTAKE_PERCENT = 0.5;
        
        public ElevatorWristState MID_CONE = new ElevatorWristState(10, 0.2);
        public double MID_CONE_HIGHT_BEFORE_ANGLE = 0.1;
        public double MID_CONE_OUTTAKE_PERCENT = 0.5;
        
        public ElevatorWristState HIGH_CUBE = new ElevatorWristState(10, 0.2);
        public double HIGH_CUBE_HIGHT_BEFORE_ANGLE = 0.1;
        public double HIGH_CUBE_OUTTAKE_PERCENT = 0.5;
        
        public ElevatorWristState MID_CUBE = new ElevatorWristState(10, 0.2);
        public double MID_CUBE_HIGHT_BEFORE_ANGLE = 0.1;
        public double MID_CUBE_OUTTAKE_PERCENT = 0.5;

        public ElevatorWristState STANDING_CONE = new ElevatorWristState(10, 0.2);

        public ElevatorWristState FALLEN_CONE = new ElevatorWristState(10, 0.2);

        public double INTAKE_GENERAL = 0.5;
        public double OUTTAKE_GENERAL = 0.5;

        public Double INTAKE_CUBE = INTAKE_GENERAL;
        public Double INTAKE_STANDING_CONE = INTAKE_GENERAL;
        public Double INTAKE_FALLEN_CONE = INTAKE_GENERAL;

        public Double CONE_SETTLE_TIME = 0.3;

        //Vison
        public double HIGH_NODE_LIMELIGHT_DIST = 1.392423;
        public double MID_TARGET_LIMELIGHT_DIST = 0.845737;

        public double HIGH_NODE_LIMELIGHT_ALIGN_OFFSET = 0.0;
        public double MID_NODE_LIMELIGHT_ALIGN_OFFSET = 0.0;

        public double LIMELIGHT_ALIGN_Y_TOLERANCE = 0.0;
        public double LIMELIGHT_ALIGN_X_TOLERANCE = 0.0;

        public double CUBE_ALIGN_Y_TOLERANCE = 0.0;
        public double CUBE_ALIGN_X_TOLERANCE = 0.0;

        public double CUBE_ALIGN_OFFSET = 0.2;
        public double CUBE_ALIGN_X_OFFSET = 0.2;

        public double CHARGING_STATION_ALIGN_OFFSET = 0.2;
    }
    public static FieldConstants HOME_FIELD = new FieldConstants();

    


    
    public static FieldConstants CurrentField = HOME_FIELD;
}
