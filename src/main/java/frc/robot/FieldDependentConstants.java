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
        public ElevatorWristState HIGH_CONE = new ElevatorWristState(107, 1.31);
        public double HIGH_CONE_HIGHT_BEFORE_ANGLE = 0.1;
        public double HIGH_CONE_OUTTAKE_PERCENT = 1;
        
        public ElevatorWristState MID_CONE = new ElevatorWristState(103.28, 0.76);
        public double MID_CONE_HIGHT_BEFORE_ANGLE = 0.02;
        public double MID_CONE_OUTTAKE_PERCENT = 0.5;
        
        public ElevatorWristState HIGH_CUBE = new ElevatorWristState(85, 0.98);
        public double HIGH_CUBE_HIGHT_BEFORE_ANGLE = 0.2;
        public double HIGH_CUBE_OUTTAKE_PERCENT = 0.3;
        

        public ElevatorWristState MID_CUBE = new ElevatorWristState(64.67, 0.41);
        public double MID_CUBE_HIGHT_BEFORE_ANGLE = 0.1;
        public double MID_CUBE_OUTTAKE_PERCENT = 0.4;

        public ElevatorWristState STANDING_CONE = new ElevatorWristState(101.63, 0.1);




        public ElevatorWristState CUBE_THROW = new ElevatorWristState(76.63, 0.37);
        public double CUBE_THROW_DELAY = 0.085;

        public ElevatorWristState FALLEN_CONE = new ElevatorWristState(
            126.52, 0.09);

        public double INTAKE_GENERAL = 0.8;
        public double OUTTAKE_GENERAL = 0.5;

        public double INTAKE_CUBE = 0.5;
        public double INTAKE_STANDING_CONE = INTAKE_GENERAL;
        public double INTAKE_FALLEN_CONE = INTAKE_GENERAL;

        public double CONE_SETTLE_TIME = 0.3;

        // //Vison

        public double HIGH_NODE_LIMELIGHT_ALIGN_OFFSET =  1.577160;
        public double MID_NODE_LIMELIGHT_ALIGN_OFFSET = 0.852018;
        public double HIGH_NODE_LIMELIGHT_FIRST_ALIGN_OFFSET = 1.8 ;// 1.552556;
        public double MID_NODE_LIMELIGHT_FIRST_ALIGN_OFFSET = 1.1;//0.913931;


        public double LIMELIGHT_ALIGN_LEFT_BOUND =  0.063;
        public double LIMELIGHT_ALIGN_LEFT_HIGH_Y_OFFSET = -0.073;
        public double LIMELIGHT_ALIGN_LEFT_MID_Y_OFFSET = -0.021;

        public double LIMELIGHT_ALIGN_RIGHT_BOUND = -0.042;
        public double LIMELIGHT_ALIGN_RIGHT_HIGH_Y_OFFSET = 0.062;
        public double LIMELIGHT_ALIGN_RIGHT_MID_Y_OFFSET = 0.0664;
        

        public double CONE_OFFSET = 0.096000;
        public double CONE_BACKUP_OFFSET = 1;

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


    public static FieldConstants COLORADO = new FieldConstants();
    static {
        COLORADO.HIGH_NODE_LIMELIGHT_ALIGN_OFFSET = 1.473441;
        COLORADO.MID_NODE_LIMELIGHT_ALIGN_OFFSET = 0.891901;
        COLORADO.HIGH_NODE_LIMELIGHT_FIRST_ALIGN_OFFSET = 1.75;
        COLORADO.MID_NODE_LIMELIGHT_FIRST_ALIGN_OFFSET = 1.05;

        COLORADO.CUBE_ALIGN_OFFSET = 0.79;
        COLORADO.CUBE_FIRST_ALIGN_OFFSET = 0.82;

        COLORADO.LIMELIGHT_ALIGN_LEFT_BOUND =  0.047;
        COLORADO.LIMELIGHT_ALIGN_LEFT_HIGH_Y_OFFSET = -0.045862;
        COLORADO.LIMELIGHT_ALIGN_LEFT_MID_Y_OFFSET = -0.047894;

        COLORADO.LIMELIGHT_ALIGN_RIGHT_BOUND = -0.03;
        COLORADO.LIMELIGHT_ALIGN_RIGHT_HIGH_Y_OFFSET = 0.075350;
        COLORADO.LIMELIGHT_ALIGN_RIGHT_MID_Y_OFFSET = 0.051564;
    }
    
    public static FieldConstants PRACTICE_FELID = new FieldConstants();
    static {
    
    }


    
    public static FieldConstants CurrentField = HOME_FIELD;
}
