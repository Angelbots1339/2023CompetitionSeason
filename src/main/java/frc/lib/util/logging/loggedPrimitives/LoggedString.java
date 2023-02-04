// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.logging.loggedPrimitives;

import java.util.function.Supplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.StringLogEntry;

import edu.wpi.first.wpilibj.DataLogManager;
import frc.lib.util.logging.LoggedContainer;
import frc.lib.util.logging.Logger.LoggingLevel;
import frc.lib.util.logging.loggedObjects.LoggedObject;

/** Add your docs here. */
public class LoggedString extends LoggedPrimitive<String> {
    private StringLogEntry logEntry;

    /**
     * This constructor is used to create a LoggedPrimitive that is an shuffulboard only LOG
     */
    public LoggedString(LoggedObject<?> object, Supplier<String> supplier, GenericEntry entry) {
        super(object, supplier, entry);
    }
    /**
     * This constructor is used to create a LoggedPrimitive that is an onboard only LOG
     */
    public LoggedString(LoggedObject<?> object, String name, Supplier<String> supplier) {
        super(object, name, supplier);
    }
    public LoggedString(String name, LoggingLevel level, LoggedContainer subsystem) {
        super(name, level, subsystem);
    }

    public LoggedString(String name, LoggingLevel level, LoggedContainer subsystem, Supplier<String> supplier) {
        super(name, level, subsystem, supplier);
    }

    @Override
    protected void initializeOnboardLog(String name, String prefix) {
        logEntry = new StringLogEntry(DataLogManager.getLog(), getOnboardLogName(name, prefix));

    }

    @Override
    protected void logOnboard(long timestamp, String value) {
        logEntry.append(value, timestamp);

    }

    @Override
    protected void logShuffleboard(String value) {
        shuffleboardEntry.setString(value);
    }

}
