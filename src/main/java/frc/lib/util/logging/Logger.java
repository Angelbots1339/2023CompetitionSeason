// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.logging;

import java.util.ArrayList;
import java.util.List;

/** Add your docs here. */
public class Logger implements Iloggable {

    private static Logger logger;
    private List<LoggedContainer> subsystems = new ArrayList<LoggedContainer>();

    public static Logger getInstance() {
        if (logger == null) {
            logger = new Logger();
        }
        return logger;
    }

    public void addContainer(LoggedContainer subsystemLogger) {
        subsystems.add(subsystemLogger);
    }
    public void removeContainer(LoggedContainer subsystemLogger) {
        subsystems.remove(subsystemLogger);
    }

    public void log(long timestamp) {
        subsystems.forEach(logger -> logger.log(0));

    }

    public enum LoggingLevel {
        SHUFFLEBOARD,
        ONBOARD_ONLY,
        NONE
    }

}
