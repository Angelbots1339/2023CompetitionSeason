// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.logging;

import java.util.HashMap;
import java.util.Map;

import frc.lib.util.logging.Logger.LoggingLevel;
import frc.lib.util.logging.loggedObjects.LoggedObject;

/** Add your docs here. */
public class LoggedSubsystem extends LoggedContainer{
    private Map<String, LoggedObject<?>> loggedObjects = new HashMap<String, LoggedObject<?>>();

    public LoggedSubsystem(String name, Map<String, LoggingLevel> logPriorities) {
        super(name, logPriorities);
    }

    @Override
    public void log(long timestamp) {
        loggedObjects.forEach((name, loggedObjects) -> loggedObjects.log(timestamp));
        super.log(timestamp);
    }
    public void add(LoggedObject<?> loggable) {
        loggedObjects.put(loggable.getName(), loggable);
    }
    

}
