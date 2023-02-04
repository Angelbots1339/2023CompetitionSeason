// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.logging.loggedObjects;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import frc.lib.util.logging.Iloggable;
import frc.lib.util.logging.LoggedContainer;
import frc.lib.util.logging.Logger.LoggingLevel;
import frc.lib.util.logging.loggedPrimitives.LoggedBoolean;
import frc.lib.util.logging.loggedPrimitives.LoggedDouble;
import frc.lib.util.logging.loggedPrimitives.LoggedDoubleArray;
import frc.lib.util.logging.loggedPrimitives.LoggedInteger;
import frc.lib.util.logging.loggedPrimitives.LoggedPrimitive;
import frc.lib.util.logging.loggedPrimitives.LoggedString;

/** Add your docs here. */
public abstract class LoggedObject<T> implements Iloggable {
    private List<LoggedPrimitive<?>> onBoard = new ArrayList<LoggedPrimitive<?>>();

    protected final LoggingLevel level;
    protected final String name;
    protected final String prefix;

    protected T object;
    private final String tabName;


    public LoggedObject(String name, LoggedContainer subsystemLogger, String logType, String tabName){
        this.level = subsystemLogger.getLoggingLevel(logType);
        this.name   = name;
        this.prefix = subsystemLogger.getName();
        this.tabName = subsystemLogger.getName() + ":" + logType;
    }

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

    
    protected void addDoubleToOnboardLog(String name, Supplier<Double> supplier){
        add(new LoggedDouble(this, name, supplier));
    }
    protected void addIntegerToOnboardLog(String name, Supplier<Integer> supplier){
        add(new LoggedInteger(this, name, supplier));
    }
    protected void addStringToOnboardLog(String name, Supplier<String> supplier){
        add(new LoggedString(this, name, supplier));
    }
    protected void addBooleanToOnboardLog(String name, Supplier<Boolean> supplier){
        add(new LoggedBoolean(this, name, supplier));
    }
    protected void addDoubleArrayToOnboardLog(String name, Supplier<double[]> supplier){
        add(new LoggedDoubleArray(this, name, supplier));
    }
    protected void addDoubleToShuffleboard(String name, Supplier<Double> supplier, ShuffleboardContainer container){
        add(new LoggedDouble(this, supplier, container.add(name, 0.0).getEntry()));
    }
    protected void addIntegerToShuffleboard(String name, Supplier<Integer> supplier, ShuffleboardContainer container){
        add(new LoggedInteger(this, supplier, container.add(name, 0).getEntry()));
    }
    protected void addStringToShuffleboard(String name, Supplier<String> supplier, ShuffleboardContainer container){
        add(new LoggedString(this, supplier, container.add(name, "").getEntry()));
    }
    protected void addBooleanToShuffleboard(String name, Supplier<Boolean> supplier, ShuffleboardContainer container){
        add(new LoggedBoolean(this, supplier, container.add(name, false).getEntry()));
    }
    protected void addDoubleArrayToShuffleboard(String name, Supplier<double[]> supplier, ShuffleboardContainer container){
        add(new LoggedDoubleArray(this, supplier, container.add(name, new double[0]).getEntry()));
    }

    protected void add(LoggedPrimitive<?> primitive){
        onBoard.add(primitive);
    }

    public void log(long timestamp) {
        for (LoggedPrimitive<?> primitive : onBoard) {
            primitive.log(timestamp);
        }
    }
    public String getName(){
        return name;
    }
    public String getPrefix(){
        return prefix;
    }
    public LoggingLevel getLevel(){
        return level;
    }
    protected ShuffleboardTab getTab(){
        System.out.println(tabName);
        return Shuffleboard.getTab(tabName);
    }

}
