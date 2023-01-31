// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.logging.loggedObjects;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import frc.lib.util.logging.LoggedContainer;

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
        ShuffleboardLayout layout = getTab().getLayout(name, BuiltInLayouts.kList);
        layout.addDouble("Yaw", () -> object.getYaw());
        layout.addDouble("Pitch", () -> object.getPitch());
        layout.addDouble("Roll", () -> object.getRoll());

        layout.addDouble("Accel x", () -> {
            short[] accel = new short[3];
            object.getBiasedAccelerometer(accel);
            return accel[0];

        });
        layout.addDouble("Accel y", () -> {
            short[] accel = new short[3];
            object.getBiasedAccelerometer(accel);
            return accel[1];
        });

    }

    @Override
    protected void initializeDataLog() {
        addDoubleToOnboardLog("Yaw", () -> object.getYaw());
        addDoubleToOnboardLog("Pitch", () -> object.getPitch());
        addDoubleToOnboardLog("Roll", () -> object.getRoll());
        short[] accel = new short[3];

        object.getBiasedAccelerometer(accel);
        addDoubleToOnboardLog("Accel x", () -> (double) accel[0]);
        addDoubleToOnboardLog("Accel y", () -> (double) accel[1]);

    }

}
