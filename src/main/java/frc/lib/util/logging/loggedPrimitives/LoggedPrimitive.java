// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.logging.loggedPrimitives;

import java.util.function.Supplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import frc.lib.util.logging.Iloggable;
import frc.lib.util.logging.LoggedContainer;
import frc.lib.util.logging.Logger.LoggingLevel;

/** Add your docs here. */
public abstract class LoggedPrimitive<T> implements Iloggable {

    protected final LoggingLevel level;

    protected GenericEntry shuffleboardEntry;
    private Supplier<T> supplier;
    private boolean isSupplied;

    public LoggedPrimitive(String name, LoggingLevel level, String prefix) {
        this.level = level;
        isSupplied = false;
        if (level == LoggingLevel.SHUFFLEBOARD) {
            shuffleboardEntry = Shuffleboard.getTab(prefix).add(name, 0).getEntry();
        } else if (level == LoggingLevel.ONBOARD_ONLY) {
            initializeOnboardLog(name, prefix);
        }
    }

    public LoggedPrimitive(String name, LoggingLevel level, String prefix, Supplier<T> supplier) {
        this(name, level, prefix);
        isSupplied = true;
        this.supplier = supplier;
    }

    public LoggedPrimitive(String name, LoggingLevel level, LoggedContainer subsystem) {
        this(name, level, subsystem.getName());
    }

    public LoggedPrimitive(String name, LoggingLevel level, LoggedContainer subsystem, Supplier<T> supplier) {
        this(name, level, subsystem.getName(), supplier);
    }

    @Override
    public void log(long timestamp) {
        if (!isSupplied)
            return;
        if (level == LoggingLevel.SHUFFLEBOARD) {
            logShuffleboard(supplier.get());
        } else if (level == LoggingLevel.ONBOARD_ONLY) {
            logOnboard(timestamp, supplier.get());
        }
    }

    public void log(long timestamp, T value) {
        if (level == LoggingLevel.SHUFFLEBOARD) {
            logShuffleboard(value);
        } else if (level == LoggingLevel.ONBOARD_ONLY) {
            logOnboard(0, value);
        }
    }

    protected String getOnboardLogName(String name, String prefix){
        return prefix + "/" + name;
    }

    protected abstract void initializeOnboardLog(String name, String prefix);

    protected abstract void logOnboard(long timestamp, T value);

    protected abstract void logShuffleboard(T value);

}
