// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.math;

/** Add your docs here. */
public class FeedforwardUtil {

    /**
     * Simple only ks feed for position loops
     * @param error error of the loop used to deturmen sign 
     * @param Ks static friction value
     * @return Ks with proper sign 
     */
    public static double positionFeedForward(double error, double Ks){
        return Math.signum(error) * Math.abs(Ks);
    }

}
