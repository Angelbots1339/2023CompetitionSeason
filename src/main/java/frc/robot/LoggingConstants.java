// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import frc.lib.util.logging.Logger.LoggingLevel;

/** Add your docs here. */
public final class LoggingConstants {
        public static final Map<String, LoggingLevel> SWERVE = Map.of(
                        "Motor", LoggingLevel.NONE,
                        "Gyro", LoggingLevel.SHUFFLEBOARD,
                        "Module", LoggingLevel.NONE,
                        "Pose", LoggingLevel.NONE,
                        "AngularDrive", LoggingLevel.NONE,
                        "Drive", LoggingLevel.NONE,
                        "Default", LoggingLevel.NONE,
                        "PidPose", LoggingLevel.NONE,
                        "PoseEstimator", LoggingLevel.SHUFFLEBOARD);

        public static final Map<String, LoggingLevel> ROBOT_CONTAINER = Map.of(
                        "Drive values", LoggingLevel.NONE,
                        "Default", LoggingLevel.NONE);

        public static final Map<String, LoggingLevel> ELEVATOR = Map.of(
                        "Drive values", LoggingLevel.NONE,
                        "Default", LoggingLevel.NONE);

        public static final Map<String, LoggingLevel> GLOBAL = Map.of(
                        "Default", LoggingLevel.NONE);

        public static final boolean LOG_TO_FILE = true;

}
