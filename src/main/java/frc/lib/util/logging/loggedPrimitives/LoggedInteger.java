// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.logging.loggedPrimitives;

import java.util.function.Supplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.IntegerLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.lib.util.logging.LoggedContainer;
import frc.lib.util.logging.Logger.LoggingLevel;
import frc.lib.util.logging.loggedObjects.LoggedObject;

/** Add your docs here. */
public class LoggedInteger extends LoggedPrimitive<Integer>{

    private IntegerLogEntry logEntry;
    
   /**
     * This constructor is used to create a LoggedPrimitive that is an shuffulboard only LOG
     */
    public LoggedInteger(LoggedObject<?> object,  Supplier<Integer> supplier, GenericEntry entry) {
        super(object, supplier, entry);
    }
    /**
     * This constructor is used to create a LoggedPrimitive that is an onboard only LOG
     */
    public LoggedInteger(LoggedObject<?> object, String name, Supplier<Integer> supplier) {
        super(object, name, supplier);
    }
    public LoggedInteger(String name, LoggingLevel level, LoggedContainer subsystem) {
        super(name, level, subsystem);
    }
    public LoggedInteger(String name, LoggingLevel level, LoggedContainer subsystem, Supplier<Integer> supplier) {
        super(name, level, subsystem, supplier);
    }

    

    


    @Override
    protected void initializeOnboardLog(String name, String prefix) {
        logEntry = new IntegerLogEntry(DataLogManager.getLog(), getOnboardLogName(name, prefix));
        
    }

    @Override
    protected void logOnboard(long timestamp, Integer value) {
        logEntry.append(value, timestamp);
    }

    @Override
    protected void logShuffleboard(Integer value) {
        shuffleboardEntry.setInteger(value);
    }

}
