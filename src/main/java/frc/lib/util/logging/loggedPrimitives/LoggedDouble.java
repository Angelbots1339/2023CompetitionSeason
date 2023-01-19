// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.logging.loggedPrimitives;

import java.util.function.Supplier;

import edu.wpi.first.util.datalog.DoubleLogEntry;

import edu.wpi.first.wpilibj.DataLogManager;
import frc.lib.util.logging.LoggedContainer;
import frc.lib.util.logging.Logger.LoggingLevel;

/** Add your docs here. */
public class LoggedDouble extends LoggedPrimitive<Double> {
    private DoubleLogEntry logEntry;

    public LoggedDouble(String name, LoggingLevel level, String prefix) {
        super(name, level, prefix);
    }

    public LoggedDouble(String name, LoggingLevel level, String prefix, Supplier<Double> supplier) {
        super(name, level, prefix, supplier);
    }

    public LoggedDouble(String name, LoggingLevel level, LoggedContainer subsystem) {
        super(name, level, subsystem);
    }

    public LoggedDouble(String name, LoggingLevel level, LoggedContainer subsystem, Supplier<Double> supplier) {
        super(name, level, subsystem, supplier);
    }

    @Override
    protected void logOnboard(long timestamp, Double value) {
        logEntry.append(value, timestamp);
    }

    @Override
    protected void logShuffleboard(Double value) {
        shuffleboardEntry.setDouble(value);
    }

    @Override
    protected void initializeOnboardLog(String name, String prefix) {
        logEntry = new DoubleLogEntry(DataLogManager.getLog(), getOnboardLogName(name, prefix));
    }

}
