// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;

/** Add your docs here. */
public class FieldUtil {

    public static final double blueGridAlignX = 0.82423;
    public static final double redGridAlignX = 0;

    public static final double getGridAlignX(){
        if(DriverStation.getAlliance() == Alliance.Red){
            return redGridAlignX - SwerveConstants.ALIGN_OFFSET;
        }else {
            return 1.412682;
        }
    }


    public static final double coneNodeBlueMidX = 0.80143; 
    public static final double coneNodeBlueHighX = 0.36943 ; 

    public static final double coneNodeMidZ = 0.6127750000000001; 
    public static final double coneNodeHighZ = 1.114425; 


    public static final double getConeNodeMidX(){
        if(DriverStation.getAlliance() == Alliance.Red){
            return FieldConstants.RED_ORIGIN.getX() - coneNodeBlueMidX;
        }else {
            return coneNodeBlueMidX;
        }
    }

    public static final double getConeNodeHighX(){
        if(DriverStation.getAlliance() == Alliance.Red){
            return FieldConstants.RED_ORIGIN.getX() - coneNodeBlueHighX;
        }else {
            return coneNodeBlueHighX;
        }
    }

    public static final Rotation2d getTwoardsDriverStation(){
        if(DriverStation.getAlliance() == Alliance.Red){
            return Rotation2d.fromDegrees(0);
        }else {
            return Rotation2d.fromDegrees(180);
        }
    }
    public static final Rotation2d getAwayFromDriverStation(){
        if(DriverStation.getAlliance() == Alliance.Red){
            return Rotation2d.fromDegrees(180);
        }else {
            return Rotation2d.fromDegrees(0);
        }
    }

    public static Pose2d getFirstAlignSingleSubstation(){
        Pose2d firstAlignBlue = new Pose2d(0,0,getAwayFromDriverStation());
        if(DriverStation.getAlliance() == Alliance.Red){
            return flipPose2d(firstAlignBlue);
        }else {
            return firstAlignBlue;
        }
    }
    public static Pose2d getSecondAlignSingleSubstation(){
        Pose2d secondAlignBlue = new Pose2d(0,0, Rotation2d.fromDegrees(90));
        if(DriverStation.getAlliance() == Alliance.Red){
            return flipPose2d(secondAlignBlue);
        }else {
            return secondAlignBlue;
        }
    }

    public static Pose2d flipPose2d(Pose2d pose2d){
        if(DriverStation.getAlliance() == Alliance.Red){
            return new Pose2d(16.51 - pose2d.getX(), pose2d.getY(), pose2d.getRotation());
        }else {
            return pose2d;
        }
    }
}
