// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.logging.loggedObjects;

import java.util.Arrays;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.lib.util.logging.LoggedContainer;
import frc.lib.util.logging.loggedPrimitives.LoggedDouble;
import frc.lib.util.logging.loggedPrimitives.LoggedDoubleArray;

/** Add your docs here. */
public class LoggedPigeon2 extends LoggedObject<Pigeon2> {

    public LoggedPigeon2(String name, LoggedContainer subsystemLogger, Pigeon2 object, String logType, String tabName) {
        super(name, subsystemLogger, object, logType, tabName);
    }

    public LoggedPigeon2(String name, LoggedContainer subsystemLogger, Pigeon2 object, String logType,
            Boolean SeptateTab) {
        super(name, subsystemLogger, object, logType, SeptateTab);
    }

    public LoggedPigeon2(String name, LoggedContainer subsystemLogger, Pigeon2 object, String logType) {
        super(name, subsystemLogger, object, logType);
    }

    @Override
    protected void initializeShuffleboard() {
        ShuffleboardLayout layout = getTab().getLayout(name, BuiltInLayouts.kList).withSize(1, 3);
        addDoubleToShuffleboard("Yaw", () -> object.getYaw(), layout);
        addDoubleToShuffleboard("Pitch", () -> object.getPitch(), layout);

        layout.addDoubleArray(name, () -> {
            short[] accel = new short[3];
            object.getBiasedAccelerometer(accel);
            return new double[]{accel[0],accel[1],accel[2]};
        }).withWidget(BuiltInWidgets.k3AxisAccelerometer);

    }

    @Override
    protected void initializeDataLog() { 
        addDoubleToOnboardLog("Yaw", () -> object.getYaw());
        addDoubleToOnboardLog("Pitch", () -> object.getPitch());
        addDoubleToOnboardLog("Roll", () -> object.getRoll());

        addDoubleArrayToOnboardLog("Accelerometer [x,y,z]", () -> {
            short[] accel = new short[3];
            object.getBiasedAccelerometer(accel);
            return new double[]{accel[0],accel[1],accel[2]};
        });

    }

}
