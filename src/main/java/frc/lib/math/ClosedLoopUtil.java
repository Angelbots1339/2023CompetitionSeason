// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.math;

import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class ClosedLoopUtil {

    /**
     * Simple only ks feed for position loops
     * @param error error of the loop used to determine sign 
     * @param Ks static friction value
     * @return Ks with proper sign 
     */
    public static double positionFeedForward(double error, double Ks){
        return Math.signum(error) * Math.abs(Ks);
    }

    
    public static double clampMaxEffort(double output, double max){
        return MathUtil.clamp(output, -max, max);
    }

    public static double stopAtSetPoint(double output, double error, double tolerance){
        if(Math.abs(error) < tolerance){
            return 0;
        }else{
            return output;
        }
    }

    public static boolean inSetPoint(double error, double tolerance){
        return Math.abs(error) < tolerance;
    }

}
