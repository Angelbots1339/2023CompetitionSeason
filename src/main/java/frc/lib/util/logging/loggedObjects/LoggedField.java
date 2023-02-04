// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.logging.loggedObjects;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.lib.util.logging.LoggedContainer;
import frc.lib.util.logging.Logger.LoggingLevel;

/** Add your docs here. */
public class LoggedField extends LoggedObject<Field2d> {

    private Map<String, Supplier<Pose2d>> robots = new HashMap<String, Supplier<Pose2d>>();
    private Map<String, DoubleArrayLogEntry> trajectories = new HashMap<String, DoubleArrayLogEntry>();

    public LoggedField(String name, LoggedContainer subsystemLogger, String logType, Boolean separateTab) {
        super(name, subsystemLogger, logType, name);
        object = new Field2d();
        if (level == LoggingLevel.SHUFFLEBOARD)
            initializeShuffleboard();
        else if (level == LoggingLevel.ONBOARD_ONLY)
            initializeDataLog();
    }

    @Override
    public void log(long timestamp) {
        super.log(timestamp);
        
        robots.forEach((name, pose2d) -> {
            Pose2d pose = pose2d.get();
            object.getObject(name).setPose(pose);
        });
        

    }


    public void addPose2d(String name, Supplier<Pose2d> pose2d, Boolean showOnShuffleboardWidget) {
        if (level == LoggingLevel.SHUFFLEBOARD) {

            getTab().addDoubleArray(name,
            () -> {
                        Pose2d pose = pose2d.get(); 
                        return new double[] { pose.getX(), pose.getY(), pose.getRotation().getDegrees() };
                    });
                
            if (showOnShuffleboardWidget)
                robots.putIfAbsent(name, pose2d);
        }

        else if (level == LoggingLevel.ONBOARD_ONLY)
            addDoubleArrayToOnboardLog(name,
                    () -> {
                        Pose2d pose = pose2d.get(); 
                       return new double[] { pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
                });
    }

    public void setTrajectory(String name, Trajectory trajectory, Boolean showOnShuffleboardWidget) {
        if (level == LoggingLevel.SHUFFLEBOARD) {
            if (showOnShuffleboardWidget)
                object.getObject(name).setTrajectory(trajectory);
            else {
                double[] arr = new double[trajectory.getStates().size() * 3];
                int ndx = 0;
                for (State pose : trajectory.getStates()) {
                    Translation2d translation = pose.poseMeters.getTranslation();
                    arr[ndx + 0] = translation.getX();
                    arr[ndx + 1] = translation.getY();
                    arr[ndx + 2] = pose.poseMeters.getRotation().getDegrees();
                    ndx += 3;
                }
                NetworkTableInstance.getDefault().getTable(getPrefix()).getSubTable(this.name).getDoubleArrayTopic(name)
                        .getGenericEntry().setDoubleArray(arr);
            }
        }
        else if (level == LoggingLevel.ONBOARD_ONLY){
            if(!trajectories.containsKey(name)){
                trajectories.put(name, new DoubleArrayLogEntry(DataLogManager.getLog(), getPrefix() + "/" + this.name + "/" + name));
            }
            double[] arr = new double[trajectory.getStates().size() * 3];
                int ndx = 0;
                for (State pose : trajectory.getStates()) {
                    Translation2d translation = pose.poseMeters.getTranslation();
                    arr[ndx + 0] = translation.getX();
                    arr[ndx + 1] = translation.getY();
                    arr[ndx + 2] = pose.poseMeters.getRotation().getDegrees();
                    ndx += 3;
                }
            trajectories.get(name).append(arr);
        }

    }

    @Override
    protected void initializeShuffleboard() {
        getTab().add(object);
    }

    @Override
    protected void initializeDataLog() {
        addDoubleToOnboardLog("x", () -> object.getRobotPose().getX());
        addDoubleToOnboardLog("y", () -> object.getRobotPose().getY());
        addDoubleToOnboardLog("theta", () -> object.getRobotPose().getRotation().getDegrees());
        addDoubleToOnboardLog("theta", () -> object.getRobotPose().getRotation().getDegrees());
    }
}
