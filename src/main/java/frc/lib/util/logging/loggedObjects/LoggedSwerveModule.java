// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.logging.loggedObjects;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.lib.util.logging.LoggedContainer;
import frc.robot.SwerveModule;

/** Add your docs here. */
public class LoggedSwerveModule extends LoggedObject<SwerveModule> {

    public LoggedSwerveModule(String name, LoggedContainer subsystemLogger, SwerveModule object, String logType,
            String tabName) {
        super(name, subsystemLogger, object, logType, tabName);
    }
    public LoggedSwerveModule(String name, LoggedContainer subsystemLogger, SwerveModule object, String logType) {
        super(name, subsystemLogger, object, logType);
    }
    public LoggedSwerveModule(String name, LoggedContainer subsystemLogger, SwerveModule object, String logType,
            Boolean SeparateTab) {
        super(name, subsystemLogger, object, logType, SeparateTab);
    }
    @Override
    protected void initializeShuffleboard() {
        ShuffleboardLayout layout = getTab().getLayout(name, BuiltInLayouts.kList).withSize(2, 3);
        layout.addDouble("CanCoder", () -> object.getCanCoder().getDegrees());
        layout.addDouble("Integrated", () -> object.getPosition().angle.getDegrees());
        layout.addDouble("Distance Meters", () -> object.getPosition().distanceMeters);
    }

    @Override
    protected void initializeDataLog() {
        addDoubleToOnboardLog("CanCoder", () -> object.getCanCoder().getDegrees());
        addDoubleToOnboardLog("Integrated", () -> object.getPosition().angle.getDegrees());
        addDoubleToOnboardLog("Distance Meters", () -> object.getPosition().distanceMeters);
    }

}
