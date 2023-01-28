// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.logging;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import frc.lib.util.logging.Logger.LoggingLevel;
import frc.lib.util.logging.loggedPrimitives.LoggedBoolean;
import frc.lib.util.logging.loggedPrimitives.LoggedDouble;
import frc.lib.util.logging.loggedPrimitives.LoggedInteger;
import frc.lib.util.logging.loggedPrimitives.LoggedPrimitive;
import frc.lib.util.logging.loggedPrimitives.LoggedString;
import frc.robot.LoggingConstants;

/** Add your docs here. */
public class LoggedContainer implements Iloggable {
    private Map<String, LoggedPrimitive<?>> loggedPrimitives = new HashMap<String, LoggedPrimitive<?>>();

    private final Map<String, LoggingLevel> logPriorities;
    private static final Map<String, LoggingLevel> globalLogPriorites = LoggingConstants.GLOBAL;

    protected final String name;

    public LoggedContainer(String name, Map<String, LoggingLevel> logPriorities) {
        this.name = name;
        this.logPriorities = logPriorities;
        Logger.getInstance().addContainer(this);

    }

    public void log(long timestamp) {
        loggedPrimitives.forEach((name, loggedPrimitive) -> loggedPrimitive.log(timestamp));
    }

    

    public void addString(String name, Supplier<String> supplier, String logType) {
        loggedPrimitives.put(name, new LoggedString(name, getLoggingLevel(logType), this, supplier));
    }
    public void addDouble(String name, Supplier<Double> supplier, String logType) {
        loggedPrimitives.put(name, new LoggedDouble(name, getLoggingLevel(logType), this, supplier));
    }
    public void addBoolean(String name, Supplier<Boolean> supplier, String logType) {
        loggedPrimitives.put(name, new LoggedBoolean(name, getLoggingLevel(logType), this, supplier));
    }
    public void addInteger(String name, Supplier<Integer> supplier, String logType) {
        loggedPrimitives.put(name, new LoggedInteger(name, getLoggingLevel(logType), this, supplier));
    }

    public void updateDouble(String name, double value, String logType) {
        loggedPrimitives.putIfAbsent(name, new LoggedDouble(name, getLoggingLevel(logType), this));
        ((LoggedDouble) loggedPrimitives.get(name)).log(0, value);
    }
    public void updateBoolean(String name, boolean value, String logType) {
        loggedPrimitives.putIfAbsent(name, new LoggedBoolean(name, getLoggingLevel(logType), this));
        ((LoggedBoolean) loggedPrimitives.get(name)).log(0, value);
    }
    public void updateString(String name, String value, String logType) {
        loggedPrimitives.putIfAbsent(name, new LoggedString(name, getLoggingLevel(logType), this));
        ((LoggedString) loggedPrimitives.get(name)).log(0, value);
    }
    public void updateInteger(String name, int value, String logType) {
        loggedPrimitives.putIfAbsent(name, new LoggedInteger(name, getLoggingLevel(logType), this));
        ((LoggedInteger) loggedPrimitives.get(name)).log(0, value);
    }

    public LoggingLevel getLoggingLevel(String logType) {
        if (logPriorities.containsKey(logType))
            return logPriorities.get(logType);
        if (globalLogPriorites.containsKey(logType))
            return globalLogPriorites.get(logType);
        if (logPriorities.containsKey("Default"))
            return logPriorities.get("Default");
        return globalLogPriorites.get("Default");
    }

    public boolean isLogging(String logType){
        return getLoggingLevel(logType) != LoggingLevel.NONE;
    }

    public String getName() {
        return name;
    }

}
