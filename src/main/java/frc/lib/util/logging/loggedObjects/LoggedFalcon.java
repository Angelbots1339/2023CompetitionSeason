// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.logging.loggedObjects;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.lib.util.logging.LoggedContainer;

/** Add your docs here. */
public class LoggedFalcon extends LoggedObject<TalonFX> {

    public LoggedFalcon(String name, LoggedContainer subsystemLogger, TalonFX object, String logType,
            String tabName) {
        super(name, subsystemLogger, object, logType, tabName);
    }

    public LoggedFalcon(String name, LoggedContainer subsystemLogger, TalonFX object, String logType,
            Boolean SeprateTab) {
        super(name, subsystemLogger, object, logType, SeprateTab);
    }

    public LoggedFalcon(String name, LoggedContainer subsystemLogger, TalonFX object, String logType) {
        super(name, subsystemLogger, object, logType);
    }

    @Override
    public void initializeShuffleboard() {
        ShuffleboardLayout layout = getTab().getLayout(name, BuiltInLayouts.kList);
        System.out.println(getTab().getTitle());
    
        layout.add("title", 0).withSize(1, 2).withPosition(1, 1).getEntry();
        layout.addNumber(name + ":Stator Current", () -> object.getStatorCurrent()).withSize(2, 2);
        layout.addNumber(name + ":Supply Current", () -> object.getSupplyCurrent()).withSize(2, 2);
    }

    @Override
    public void initializeDataLog() {
        addDoubleToOnboardLog("Stator Current", () -> object.getStatorCurrent());
        addDoubleToOnboardLog("Supply Current", () -> object.getSupplyCurrent());
    }



}
