// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;

import frc.lib.util.logging.Logger.LoggingLevel;

/** Add your docs here. */
public final class LoggingConstants {
        public static final Map<String, LoggingLevel> SWERVE = Map.of(
                        "Motor", LoggingLevel.ONBOARD_ONLY,
                        "Gyro", LoggingLevel.NONE,
                        "Module", LoggingLevel.ONBOARD_ONLY,
                        "Pose", LoggingLevel.NONE,
                        "AngularDrive", LoggingLevel.NONE,
                        "Drive", LoggingLevel.NONE,
                        "Default", LoggingLevel.NONE,
                        "PidPose", LoggingLevel.NONE,
                        "Auto", LoggingLevel.NONE,
                        "PoseEstimator", LoggingLevel.ONBOARD_ONLY);

        public static final Map<String, LoggingLevel> ROBOT_CONTAINER = Map.of(
                        "Drive values", LoggingLevel.NONE,
                        "Default", LoggingLevel.NONE);

        public static final Map<String, LoggingLevel> ELEVATOR = Map.of(
                        "Main", LoggingLevel.ONBOARD_ONLY,
                        "Motion Magic", LoggingLevel.NONE,
                        "Default", LoggingLevel.NONE);
        public static final Map<String, LoggingLevel> WRIST = Map.of(
                        "Main", LoggingLevel.ONBOARD_ONLY,
                        "Motion Magic", LoggingLevel.NONE,
                        "Default", LoggingLevel.NONE);
        public static final Map<String, LoggingLevel> INTAKE = Map.of(
                                "Main", LoggingLevel.ONBOARD_ONLY,
                                "Motion Magic", LoggingLevel.NONE,
                                "Default", LoggingLevel.NONE);

        public static final Map<String, LoggingLevel> GLOBAL = Map.of(
                        "Default", LoggingLevel.NONE);


}
