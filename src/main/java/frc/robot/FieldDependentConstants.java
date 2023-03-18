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
        public PolynomialRegression MID_CONE_REGRESSION = ConeOffsetRegression.midPracticeFieldReg;
        public PolynomialRegression HIGH_CONE_REGRESSION = ConeOffsetRegression.highPracticeFieldReg;
        
        //intake positions
        public ElevatorWristState HIGH_CONE = new ElevatorWristState(110, 1.31);
        public double HIGH_CONE_HIGHT_BEFORE_ANGLE = 0.1;
        public double HIGH_CONE_OUTTAKE_PERCENT = 0.8;
        
        public ElevatorWristState MID_CONE = new ElevatorWristState(103.28, 0.76);
        public double MID_CONE_HIGHT_BEFORE_ANGLE = 0.2;
        public double MID_CONE_OUTTAKE_PERCENT = 0.5;
        
        public ElevatorWristState HIGH_CUBE = new ElevatorWristState(85, 0.98);
        public double HIGH_CUBE_HIGHT_BEFORE_ANGLE = 0.2;
        public double HIGH_CUBE_OUTTAKE_PERCENT = 0.5;
        

        public ElevatorWristState MID_CUBE = new ElevatorWristState(64.67, 0.41);
        public double MID_CUBE_HIGHT_BEFORE_ANGLE = 0.1;
        public double MID_CUBE_OUTTAKE_PERCENT = 0.5;

        public ElevatorWristState STANDING_CONE = new ElevatorWristState(101.63, 0.027);

        public ElevatorWristState FALLEN_CONE = new ElevatorWristState(
            118.52, 0.02);

        public double INTAKE_GENERAL = 0.5;
        public double OUTTAKE_GENERAL = 0.5;

        public Double INTAKE_CUBE = 0.5;
        public Double INTAKE_STANDING_CONE = INTAKE_GENERAL;
        public Double INTAKE_FALLEN_CONE = INTAKE_GENERAL;

        public Double CONE_SETTLE_TIME = 0.3;

        // //Vison

        public double HIGH_NODE_LIMELIGHT_ALIGN_OFFSET =  1.557160;
        public double MID_NODE_LIMELIGHT_ALIGN_OFFSET = 0.842018;
        public double HIGH_NODE_LIMELIGHT_FIRST_ALIGN_OFFSET = 1.8 ;// 1.552556;
        public double MID_NODE_LIMELIGHT_FIRST_ALIGN_OFFSET = 1.1;//0.913931;

        public double CONE_OFFSET = 0.07;
        public double CONE_BACKUP_OFFSET = 0.02;

        public double LIMELIGHT_ALIGN_Y_TOLERANCE = 0.02;

        public double CUBE_ALIGN_Y_TOLERANCE = 0.02;

        public double CHARGE_STATION_MIN_ANGLE = 2;
        public double CHARGE_STATION_MAX_ANGLE = 10;
        public double CHARGE_STATION_ALIGN_MIN_TIME = 0.1;


        

        public double CUBE_ALIGN_OFFSET = 0.684157;
        public double CUBE_FIRST_ALIGN_OFFSET = 0.8;

        public double CHARGING_STATION_ALIGN_OFFSET = 16.540988 - 12.450292648584131;
    }
    public static FieldConstants HOME_FIELD = new FieldConstants();

    public static FieldConstants ST_LUIS = new FieldConstants();
    static {
        ST_LUIS.MID_CONE_REGRESSION = ConeOffsetRegression.midCompFieldReg;
        ST_LUIS.HIGH_CONE_REGRESSION = ConeOffsetRegression.highCompFieldReg;

        ST_LUIS.HIGH_NODE_LIMELIGHT_ALIGN_OFFSET =  1.385142;
        ST_LUIS.MID_NODE_LIMELIGHT_ALIGN_OFFSET = 0.903931;
        ST_LUIS.HIGH_NODE_LIMELIGHT_FIRST_ALIGN_OFFSET = 1.457768;
        ST_LUIS.MID_NODE_LIMELIGHT_FIRST_ALIGN_OFFSET = 0.924321;
        ST_LUIS.CUBE_ALIGN_OFFSET = 0.79;
        ST_LUIS.CUBE_FIRST_ALIGN_OFFSET = 0.82;



    
    }
    


    
    public static FieldConstants CurrentField = HOME_FIELD;
}
