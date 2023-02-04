
package frc.lib.util.logging.loggedPrimitives;

import java.util.function.Supplier;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.lib.util.logging.LoggedContainer;
import frc.lib.util.logging.Logger.LoggingLevel;
import frc.lib.util.logging.loggedObjects.LoggedObject;

public class LoggedDoubleArray extends LoggedPrimitive<double[]>  {
  private DoubleArrayLogEntry logEntry;
  /**
     * This constructor is used to create a LoggedPrimitive that is an shuffulboard only LOG
     */
  public LoggedDoubleArray(LoggedObject<?> object, Supplier<double[]> supplier, GenericEntry entry) {
    super(object, supplier, entry);
  }
  /**
   * This constructor is used to create a LoggedPrimitive that is an onboard only LOG
   */ 
  public LoggedDoubleArray(LoggedObject<?> object, String name, Supplier<double[]> supplier) {
    super(object, name, supplier);
  }
  public LoggedDoubleArray(String name, LoggingLevel level, LoggedContainer subsystem, Supplier<double[]> supplier) {
    super(name, level, subsystem, supplier);
  }

  public LoggedDoubleArray(String name, LoggingLevel level, LoggedContainer subsystem) {
    super(name, level, subsystem);
  }


  @Override
  protected void initializeOnboardLog(String name, String prefix) {
    logEntry = new DoubleArrayLogEntry(DataLogManager.getLog(), getOnboardLogName(name, prefix));
  }

  @Override
  protected void logOnboard(long timestamp, double[] value) {
    logEntry.append(value, timestamp);
  }

  @Override
  protected void logShuffleboard(double[] value) {
    shuffleboardEntry.setDoubleArray(value);
  }
  
}
