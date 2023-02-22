// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class FelidUtil {

    public static final double blueGridAlignX = 0.82423;
    public static final double redGridAlignX = 0;


    public static final double coneNodeBlueMidX = 0.58928; //A mid point between the two nodes used for estimation of y offset when heigh of node is unknown
    public static final double coneNodeRedMidX = 0;



    public static final double getGridAlignX(Alliance alliance){
        if(alliance == Alliance.Red){
            return redGridAlignX - SwerveConstants.ALIGN_OFFSET;
        }else {
            return 1.775240;
        }
    }

    public static final double getConeNodeMidX(Alliance alliance){
        if(alliance == Alliance.Red){
            return coneNodeRedMidX;
        }else {
            return coneNodeBlueMidX;
        }
    }

}
