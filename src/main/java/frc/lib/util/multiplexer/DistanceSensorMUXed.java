// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util.multiplexer;

import static frc.robot.Constants.MultiplexerConstants.*;

import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.jni.VL53L0XJNI;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C.Port;

/** Add your docs here. */
public class DistanceSensorMUXed {
    private final int i2cPort;
    private final int port;
    private RangeProfile profile = RangeProfile.kDefault;
    private final int addr = 0x53;
    private boolean enabled;
    private Rev2mDistanceSensor rev2mDistanceSensor;

    public DistanceSensorMUXed(int port, RangeProfile rangeProfile) {
        this.port = port;
        if (DEFAULT_PORT == Port.kOnboard)
            i2cPort = 0;
        else
            i2cPort = 1;

        if (Multiplexer.getInstance().setDevice(port)) {
            VL53L0XJNI.Init(i2cPort, addr);
            if (!initialize()) {
                DriverStation.reportError(String.format("Error initializing Rev 2M device on " +
                        "%s. Please check your connections",
                        "i2c: " + (DEFAULT_PORT == Port.kMXP ? "MXP:" : "Onboard:") + "MUX port: " + port), false);
            }
        }

    }

    
    public double getRangeMillimeter() {
        double range = -1;
        if (Multiplexer.getInstance().setDevice(port) && enabled) {
            if (VL53L0XJNI.GetMeasurementDataReady(i2cPort, addr))
                range = VL53L0XJNI.GetRangingMeasurementData(i2cPort, addr);
        }
        if (range > 0)
            return range;
        else
            return -1;
    }

    /**
     * @return meters
     */
    public double getRange() {
        return getRangeMillimeter() / 1000.0;
    }



    public boolean getEnabled() {
        return enabled;
    }

    private boolean initialize() {
        boolean status = false;
        int statusInt;

        if ((statusInt = VL53L0XJNI.ValidateI2C(i2cPort, addr)) != 0) {
            DriverStation.reportError(String.format("Error 0x%08X: Could not communicate" +
                    " with Rev 2M sensor over I2C.",
                    statusInt), false);
        }

        status = VL53L0XJNI.DataInit(i2cPort, addr);

        if (status)
            status = VL53L0XJNI.GetDeviceInfo(i2cPort, addr);

        if (status)
            status = VL53L0XJNI.StaticInit(i2cPort, addr);

        if (status)
            status = VL53L0XJNI.PerformRefCalibration(i2cPort, addr);

        if (status)
            status = VL53L0XJNI.PerformRefSpadManagement(i2cPort, addr);

        if (status)
            status = VL53L0XJNI.SetDeviceMode(1, i2cPort, addr);

        if (status) {
            status = setRangeProfile(RangeProfile.kDefault);
            if (status) {
                enabled = true;
            }
        }
        if (status) {
            status = VL53L0XJNI.StartMeasurement(i2cPort, addr);
        }

        return status;
    }

    /**
     * Sets the range profile for the sensor. Valid options are:
     * kDefault
     * kHighAccuracy
     * kLongRange
     * kHighSpeed
     *
     * Range profiles are based on timing budgets defined in the VL53L0X
     * datasheet
     *
     * If called in automatic mode, the sensor will be stopped until the
     * new profile is set. Measurements will not be available during this
     * period
     *
     * @param profile The range profile to set in the sensor
     *
     * @return Profile successfully changed
     */
    public boolean setRangeProfile(RangeProfile profile) {
        if (this.profile == profile)
            return true; // ignore the case of no change
        if (profile == RangeProfile.kHighAccuracy)
            return setProfileHighAccuracy();
        else if (profile == RangeProfile.kLongRange)
            return setProfileLongRange();
        else if (profile == RangeProfile.kHighSpeed)
            return setProfileHighSpeed();
        else
            return setProfileDefault();
    }

    private boolean setProfileLongRange() {
        // System.out.println("Setting profile to long range");

        boolean status = false;
        status = VL53L0XJNI.SetLimitCheckEnableSigmaFinalRange(1, port, addr);

        if (status)
            status = VL53L0XJNI.SetLimitCheckEnableSignalRateFinalRange(1, port, addr);

        if (status)
            status = VL53L0XJNI.SetLimitCheckEnableRangeIgnoreThreshold(0, port, addr);

        if (status)
            status = VL53L0XJNI.SetLimitCheckValueSigmaFinalRange(60, port, addr);

        if (status)
            status = VL53L0XJNI.SetLimitCheckValueSignalRateFinalRange(0.1, port, addr);

        if (status)
            status = VL53L0XJNI.SetMeasurementTimingBudgetMicroSeconds(33000, port, addr);

        if (status)
            status = VL53L0XJNI.SetVcselPulsePeriodPreRange(18, port, addr);

        if (status)
            status = VL53L0XJNI.SetVcselPulsePeriodFinalRange(14, port, addr);

        if (status)
            profile = RangeProfile.kLongRange;

        return status;
    }

    private boolean setProfileHighAccuracy() {
        // System.out.println("Setting profile to high accuracy");
        boolean status = false;
        status = VL53L0XJNI.SetLimitCheckEnableSigmaFinalRange(1, port, addr);

        if (status)
            status = VL53L0XJNI.SetLimitCheckEnableSignalRateFinalRange(1, port, addr);

        if (status)
            status = VL53L0XJNI.SetLimitCheckEnableRangeIgnoreThreshold(0, port, addr);

        if (status)
            status = VL53L0XJNI.SetLimitCheckValueSigmaFinalRange(18, port, addr);

        if (status)
            status = VL53L0XJNI.SetLimitCheckValueSignalRateFinalRange(0.25, port, addr);

        if (status)
            status = VL53L0XJNI.SetMeasurementTimingBudgetMicroSeconds(200000, port, addr);

        if (status)
            status = VL53L0XJNI.SetVcselPulsePeriodPreRange(14, port, addr);

        if (status)
            status = VL53L0XJNI.SetVcselPulsePeriodFinalRange(10, port, addr);

        if (status)
            profile = RangeProfile.kHighAccuracy;

        return status;
    }

    private boolean setProfileHighSpeed() {
        // System.out.println("Setting profile to high speed");
        boolean status = false;
        status = VL53L0XJNI.SetLimitCheckEnableSigmaFinalRange(1, port, addr);

        if (status)
            status = VL53L0XJNI.SetLimitCheckEnableSignalRateFinalRange(1, port, addr);

        if (status)
            status = VL53L0XJNI.SetLimitCheckEnableRangeIgnoreThreshold(0, port, addr);

        if (status)
            status = VL53L0XJNI.SetLimitCheckValueSigmaFinalRange(32, port, addr);

        if (status)
            status = VL53L0XJNI.SetLimitCheckValueSignalRateFinalRange(0.25, port, addr);

        if (status)
            status = VL53L0XJNI.SetMeasurementTimingBudgetMicroSeconds(30000, port, addr);

        if (status)
            status = VL53L0XJNI.SetVcselPulsePeriodPreRange(14, port, addr);

        if (status)
            status = VL53L0XJNI.SetVcselPulsePeriodFinalRange(10, port, addr);

        if (status)
            profile = RangeProfile.kHighSpeed;

        return status;
    }

    private boolean setProfileDefault() {
        // System.out.println("Setting profile to default");
        boolean status = false;
        status = VL53L0XJNI.SetLimitCheckEnableSigmaFinalRange(1, port, addr);

        if (status)
            status = VL53L0XJNI.SetLimitCheckEnableSignalRateFinalRange(1, port, addr);

        if (status)
            status = VL53L0XJNI.SetLimitCheckEnableRangeIgnoreThreshold(0, port, addr);

        if (status)
            status = VL53L0XJNI.SetLimitCheckValueSigmaFinalRange(18, port, addr);

        if (status)
            status = VL53L0XJNI.SetLimitCheckValueSignalRateFinalRange(0.25, port, addr);

        if (status)
            status = VL53L0XJNI.SetMeasurementTimingBudgetMicroSeconds(33823, port, addr);

        if (status)
            status = VL53L0XJNI.SetVcselPulsePeriodPreRange(14, port, addr);

        if (status)
            status = VL53L0XJNI.SetVcselPulsePeriodFinalRange(10, port, addr);

        if (status)
            profile = RangeProfile.kDefault;

        return status;
    }
}
