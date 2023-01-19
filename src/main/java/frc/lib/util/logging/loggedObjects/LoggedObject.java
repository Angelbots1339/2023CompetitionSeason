// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.logging.loggedObjects;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.util.logging.Iloggable;
import frc.lib.util.logging.LoggedContainer;
import frc.lib.util.logging.Logger.LoggingLevel;
import frc.lib.util.logging.loggedPrimitives.LoggedDouble;
import frc.lib.util.logging.loggedPrimitives.LoggedInteger;
import frc.lib.util.logging.loggedPrimitives.LoggedString;

/** Add your docs here. */
public abstract class LoggedObject<T> implements Iloggable {
    private List<LoggedDouble> doubleLogs = new ArrayList<LoggedDouble>();
    private List<LoggedString> stringLogs = new ArrayList<LoggedString>();
    private List<LoggedInteger> integerLogs = new ArrayList<LoggedInteger>();

    protected final LoggingLevel level;
    protected final String name;
    protected final String prefix;

    protected final T object;
    private final String tabName;


    private LoggedObject(String name, LoggingLevel level, String prefix, T object, String tabName) {
        this.tabName = tabName;
        this.name = name;
        this.level = level;
        this.prefix = prefix;
        this.object = object;

        if (level == LoggingLevel.SHUFFLEBOARD)
            initializeShuffleboard();
        else if(level == LoggingLevel.ONBOARD_ONLY)
            initializeDataLog();
    }

    public LoggedObject(String name, LoggedContainer subsystemLogger, T object, String logType, String tabName) {
        this(name, subsystemLogger.getLoggingLevel(logType), subsystemLogger.getName(), object, tabName);
    }

    public LoggedObject(String name, LoggedContainer subsystemLogger, T object, String logType, Boolean SeptateTab) {
        this(name, subsystemLogger, object, logType,
                SeptateTab? subsystemLogger.getName() + ":" + logType
                        : subsystemLogger.getName());
    }

    public LoggedObject(String name, LoggedContainer subsystemLogger, T object, String logType) {
        this(name, subsystemLogger, object, logType, false);
    }

    protected abstract void initializeShuffleboard();

    protected abstract void initializeDataLog();

    public void addDoubleToOnboardLog(String name, Supplier<Double> doubleSupplier) {
        doubleLogs.add(new LoggedDouble(name, level, prefix + "/" + this.name, doubleSupplier));
    }
    public void addStringToOnboardLog(String name, Supplier<String> stringSupplier) {
        stringLogs.add(new LoggedString(name, level, prefix + "/" + this.name, stringSupplier));
    }
    public void addIntegerToOnboardLog(String name, Supplier<Integer> integerSupplier) {
        integerLogs.add(new LoggedInteger(name, level, prefix + "/" + this.name, integerSupplier));
    }

    public void log(long timestamp) {
        if (level == LoggingLevel.ONBOARD_ONLY) {
            doubleLogs.forEach((doubleLog) -> doubleLog.log(timestamp));
        }
    }
    public String getName(){
        return name;
    }
    protected ShuffleboardTab getTab(){
        System.out.println(tabName);
        return Shuffleboard.getTab(tabName);
    }

}
