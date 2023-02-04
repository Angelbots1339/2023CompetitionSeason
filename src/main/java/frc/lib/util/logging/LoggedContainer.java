// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.logging;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private static final Map<String, LoggingLevel> globalLogPriorities = LoggingConstants.GLOBAL;

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
        if(!loggedPrimitives.containsKey(name)){
            loggedPrimitives.putIfAbsent(name, new LoggedDouble(name, getLoggingLevel(logType), this));
        }
        if(loggedPrimitives.get(name) instanceof LoggedDouble){
            ((LoggedDouble) loggedPrimitives.get(name)).log(0, value);
            return;
        }
        DriverStation.reportError("Incorrect log type", null);
    }
    public void updateBoolean(String name, boolean value, String logType) {
        if(!loggedPrimitives.containsKey(name)){
            loggedPrimitives.putIfAbsent(name, new LoggedBoolean(name, getLoggingLevel(logType), this));
        }
        if(loggedPrimitives.get(name) instanceof LoggedBoolean){
            ((LoggedBoolean) loggedPrimitives.get(name)).log(0, value);
            return;
        }
        DriverStation.reportError("Incorrect log type", null);
        
    }
    public void updateString(String name, String value, String logType) {
        if(!loggedPrimitives.containsKey(name)){
            loggedPrimitives.putIfAbsent(name, new LoggedString(name, getLoggingLevel(logType), this));
        }
        if(loggedPrimitives.get(name) instanceof LoggedString){
            ((LoggedString) loggedPrimitives.get(name)).log(0, value);
            return;
        }
        DriverStation.reportError("Incorrect log type", null);
    }
    public void updateInteger(String name, int value, String logType) {
        if(!loggedPrimitives.containsKey(name)){
            loggedPrimitives.putIfAbsent(name, new LoggedInteger(name, getLoggingLevel(logType), this));
        }
        if(loggedPrimitives.get(name) instanceof LoggedInteger){
            ((LoggedInteger) loggedPrimitives.get(name)).log(0, value);
            return;
        }
        DriverStation.reportError("Incorrect log type", null);
    }

    public LoggingLevel getLoggingLevel(String logType) {
        if (logPriorities.containsKey(logType))
            return logPriorities.get(logType);
        if (globalLogPriorities.containsKey(logType))
            return globalLogPriorities.get(logType);
        if (logPriorities.containsKey("Default"))
            return logPriorities.get("Default");
        return globalLogPriorities.get("Default");
    }

    public boolean isLogging(String logType){
        return getLoggingLevel(logType) != LoggingLevel.NONE;
    }

    public String getName() {
        return name;
    }

}
