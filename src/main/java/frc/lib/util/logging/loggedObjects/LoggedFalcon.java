// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.logging.loggedObjects;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.lib.util.logging.LoggedContainer;
import frc.lib.util.logging.loggedPrimitives.LoggedDouble;

/** Add your docs here. */
public class LoggedFalcon extends LoggedObject<TalonFX> {

    public LoggedFalcon(String name, LoggedContainer subsystemLogger, TalonFX object, String logType,
            String tabName) {
        super(name, subsystemLogger, object, logType, tabName);
    }

    public LoggedFalcon(String name, LoggedContainer subsystemLogger, TalonFX object, String logType,
            Boolean SeptateTab) {
        super(name, subsystemLogger, object, logType, SeptateTab);
    }

    public LoggedFalcon(String name, LoggedContainer subsystemLogger, TalonFX object, String logType) {
        super(name, subsystemLogger, object, logType);
    }

    @Override
    public void initializeShuffleboard() {
        ShuffleboardLayout layout = getTab().getLayout(name, BuiltInLayouts.kList);
        addDoubleToShuffleboard("Stator Current", () ->  object.getStatorCurrent(), layout);
        addDoubleToShuffleboard("Supply Current", () ->  object.getSupplyCurrent(), layout);
        addDoubleToShuffleboard("Bus Voltage", () ->  object.getBusVoltage(), layout);
        addDoubleToShuffleboard("Output Voltage", () ->  object.getMotorOutputVoltage(), layout);
        addDoubleToShuffleboard("Temp", () ->  object.getTemperature(), layout);

    }

    @Override
    public void initializeDataLog() {
        addDoubleToOnboardLog("Stator Current", () -> object.getStatorCurrent());
        addDoubleToOnboardLog("Supply Current", () -> object.getSupplyCurrent());
        addDoubleToOnboardLog("Bus Voltage", () -> object.getBusVoltage());
        addDoubleToOnboardLog("Output Voltage", () -> object.getMotorOutputVoltage());
        addDoubleToOnboardLog("Temp", () -> object.getTemperature());
    }



}
