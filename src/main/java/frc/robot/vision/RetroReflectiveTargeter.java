// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.util.FieldUtil;
import frc.lib.util.LimeLight;
import frc.robot.FieldDependentConstants;
import frc.robot.FieldDependentConstants.FieldConstants;
import frc.robot.commands.objectManipulation.score.ScoreCommandFactory.ScoreHight;
import frc.robot.Constants.VisionConstants;

/** Add your docs here. */
public class RetroReflectiveTargeter {
    private static targetingStatus status = targetingStatus.NONE;

    private static double xOffset, yOffset = 0;

    public static void update(Pose2d robotPose, boolean favorHigh) {
        LimeLight.setLedMode(3);
        LimeLight.setCamMode(0);

        int highPipeLine = favorHigh ? VisionConstants.FAVOR_HIGH_HIGH_PIPELINE : VisionConstants.FAVOR_HIGH_HIGH_PIPELINE;
        int midPipeLine = favorHigh ? VisionConstants.FAVOR_HIGH_MID_PIPELINE : VisionConstants.FAVOR_HIGH_MID_PIPELINE;

        if (LimeLight.hasTargets()) {
            double horizontalOffset = LimeLight.getHorizontalOffset();
            double verticalOffset = LimeLight.getVerticalOffset();

            
            double highTargetPredictionZ = FieldUtil.coneNodeHighZ - VisionConstants.LIMELIGHT_CAM_POS.getZ();

            double midTargetPredictionZ = FieldUtil.coneNodeMidZ - VisionConstants.LIMELIGHT_CAM_POS.getZ();

            if (verticalOffset > 0) { // high
                LimeLight.setPipelineMode(highPipeLine);
                if (LimeLight.getPipeline() == highPipeLine) {
                    status = targetingStatus.HIGH;
                    xOffset = highTargetPredictionZ / Math.tan(Math.toRadians(verticalOffset));
                }
            } else { // mid
                LimeLight.setPipelineMode(midPipeLine);
                //SmartDashboard.putString("Target", "Mid");
                if (LimeLight.getPipeline() == midPipeLine) {
                    status = targetingStatus.MID;
                    xOffset = midTargetPredictionZ / Math.tan(Math.toRadians(verticalOffset));
                }
            }


            yOffset = Math.tan(Math.toRadians(horizontalOffset)) * xOffset;

            

        } else {
            status = targetingStatus.NONE;
            xOffset = -1;
            yOffset = 0;
        }
    }

    public static double getXOffset() {
        return xOffset;
    }
    
  

    public static double getYOffset() {
        return yOffset;
    }

    public static targetingStatus getStatus() {
        return status;
    }
    public static ScoreHight getScoreHight() {
        switch (status) {
            case HIGH:
                return ScoreHight.HIGH;
            case MID:
                return ScoreHight.MID;
            default:
                return ScoreHight.HIGH;
        }
    }

    public static double getYOffsetFromConeOffset(Pose2d robotPose, double coneOffset) {

        double horizontalOffset = LimeLight.getHorizontalOffset();

        switch (status) {
            case HIGH:
                return Math.tan(Math.toRadians(horizontalOffset))
                        * xOffset;
            case MID:
                return Math.tan(Math.toRadians(horizontalOffset))
                        * xOffset;
            default:
                return 0;
        }

    }

    public enum targetingStatus {
        HIGH, MID, NONE;
    }

    public static Rotation2d getAdjustedYaw(Rotation2d rotation2d) {
        if (Math.abs(rotation2d.getDegrees()) < 90) {
            return rotation2d;
        } else {
            return new Rotation2d((180 - Math.abs(rotation2d.getRadians())) * Math.signum(rotation2d.getRadians()));
        }
    }

}
