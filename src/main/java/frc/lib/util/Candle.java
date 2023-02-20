// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.util;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;
import com.ctre.phoenix.led.TwinkleAnimation.TwinklePercent;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.RobotContainer;
import static frc.robot.Constants.CandleConstants.*;

/** Add your docs here. */
public class Candle {
    private static Candle CANDLE = null;

    private CANdle candle = new CANdle(CANDLE_ID, "rio");

    // The first dimension is for individual zones.
    // Within the second dimension, the first number is the start idx for the zone,
    // and the second is the # of leds in that zone
    private int[][] ledZones = new int[4][2];

    public enum HumanPlayerCommStates {
        LeftCube,
        RightCube,
        SingleCube,

        LeftCone,
        RightCone,
        SingleCone,

    }

    private HumanPlayerCommStates currentComState = HumanPlayerCommStates.LeftCone;
    private double humanPlayerBlinkLastTime;

    public enum LEDState {
        Idle,
        Disabled,
        PreMatch,
        TestMode,
        Off,

        Fire,
        Intake,
        ReverseIntake,
        HumanPlayerCommunication,

    }

    private LEDState currentState = LEDState.Off;

    public Candle() {
        changeLedState(LEDState.Off);
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;

        candle.configAllSettings(configAll, 100);
        candle.configLOSBehavior(true);

        ledZones[0][0] = 0;
        ledZones[0][1] = 60;

        ledZones[1][0] = 60;
        ledZones[1][1] = 120;

        ledZones[2][0] = 120;
        ledZones[2][1] = 180;

        ledZones[3][0] = 180;
        ledZones[3][1] = 247;

        humanPlayerBlinkLastTime = Timer.getFPGATimestamp();
    }

    public void changeHumanPlayerComState(HumanPlayerCommStates state) {
        currentComState = state;
    }

    public void changeLedState(LEDState state) {

        for (int i = 0; i < 10; ++i) {
            candle.clearAnimation(i);
        }
        candle.setLEDs(0, 0, 0, 0, OFFSET_LENGTH, 128);
        switch (state) {
            case Idle:
                candle.animate(new TwinkleAnimation(255, 255, 255, 0, 0.4, ledZones[2][1], TwinklePercent.Percent30,
                        OFFSET_LENGTH + ledZones[1][0]), 1);

                currentState = LEDState.Idle;

                break;
            case Disabled:
                candle.animate(new LarsonAnimation(255, 255, 255, 0, 0.5, TOTAL_STRIP_LENGTH, BounceMode.Front, 3,
                        OFFSET_LENGTH));
                currentState = LEDState.Disabled;
                break;

            case PreMatch:
                currentState = LEDState.PreMatch;
                break;

            case TestMode:
                candle.animate(new SingleFadeAnimation(200, 70, 0, 0, 0.1, TOTAL_STRIP_LENGTH, OFFSET_LENGTH), 1);

                currentState = LEDState.TestMode;
                break;

            case Off:

                currentState = LEDState.Off;
                break;

            default:
                break;

            // *************** Robot-Specific Animations ***************** //
            case Fire: // TODO Tune the reverse directions
                candle.animate(
                        new FireAnimation(1, 0.7, ledZones[0][1], 0.8, 0.4, false, OFFSET_LENGTH + ledZones[0][0]), 1);
                candle.animate(
                        new FireAnimation(1, 0.7, ledZones[1][1], 0.8, 0.4, true, OFFSET_LENGTH + ledZones[1][0]), 2);
                candle.animate(
                        new FireAnimation(1, 0.7, ledZones[2][1], 0.8, 0.6, false, OFFSET_LENGTH + ledZones[2][0]), 3);
                candle.animate(
                        new FireAnimation(1, 0.7, ledZones[3][1], 0.8, 0.3, true, OFFSET_LENGTH + ledZones[3][0]), 4);
                currentState = LEDState.Fire;
                // candle.animate(new TwinkleAnimation(255, 18, 213, 0, .1, 120 + 8 - 58,
                // TwinklePercent.Percent30, 58), 3);
                break;

            case Intake:
                candle.animate(new ColorFlowAnimation(128, 20, 60, 0, 0.9, ledZones[1][1], Direction.Forward,
                        OFFSET_LENGTH + ledZones[0][0]), 1);
                candle.animate(new ColorFlowAnimation(128, 20, 60, 0, 0.9, ledZones[3][1], Direction.Backward,
                        OFFSET_LENGTH + ledZones[2][0]), 2);

                currentState = LEDState.Intake;

                break;
            case ReverseIntake:

                candle.animate(new ColorFlowAnimation(255, 20, 0, 0, 0.9, ledZones[1][1], Direction.Backward,
                        OFFSET_LENGTH + ledZones[0][0]), 1);
                candle.animate(new ColorFlowAnimation(255, 20, 0, 0, 0.9, ledZones[3][1], Direction.Forward,
                        OFFSET_LENGTH + ledZones[2][0]), 2);

                currentState = LEDState.ReverseIntake;
                break;

            case HumanPlayerCommunication:

                candle.animate(new TwinkleAnimation(255, 255, 255, 0, 0.4, ledZones[2][1], TwinklePercent.Percent30,
                OFFSET_LENGTH + ledZones[1][0]), 2);

                currentState = LEDState.HumanPlayerCommunication;
                break;
        }
    }

    private void humanPlayerComPeriodic(boolean off) {

        switch (currentComState) {

            // Cubes
            case LeftCube:
                candle.setLEDs(off ? 0 : 162, 0, off ? 0 : 255, 0,
                        OFFSET_LENGTH + ledZones[0][0],
                        ledZones[0][1]);
                break;

            case RightCube:
                candle.setLEDs(off ? 0 : 162, 0, off ? 0 : 255, 0,
                        OFFSET_LENGTH + ledZones[3][0],
                        ledZones[3][1]);
                break;

            case SingleCube:
                candle.setLEDs(off ? 0 : 162, 0, off ? 0 : 255, 0,
                        OFFSET_LENGTH + ledZones[0][0],
                        ledZones[0][1]);

                candle.setLEDs(off ? 0 : 162, 0, off ? 0 : 255, 0,
                        OFFSET_LENGTH + ledZones[3][0],
                        ledZones[3][1]);
                break;

            // Cones
            case LeftCone:
                candle.setLEDs(off ? 0 : 255, off ? 0 : 255, 0, 0,
                        OFFSET_LENGTH + ledZones[0][0],
                        ledZones[0][1]);
                break;

            case RightCone:
                candle.setLEDs(off ? 0 : 255, off ? 0 : 255, 0, 0,
                        OFFSET_LENGTH + ledZones[3][0],
                        ledZones[3][1]);
                break;

            case SingleCone:
                candle.setLEDs(off ? 0 : 255, off ? 0 : 255, 0, 0,
                        OFFSET_LENGTH + ledZones[0][0],
                        ledZones[0][1]);

                candle.setLEDs(off ? 0 : 255, off ? 0 : 255, 0, 0,
                        OFFSET_LENGTH + ledZones[3][0],
                        ledZones[3][1]);
                break;
        }

    }

    public void periodic() {

        if (currentState == LEDState.HumanPlayerCommunication) {

            if (Timer.getFPGATimestamp() - humanPlayerBlinkLastTime > HUMAN_PLAYER_COM_BLINK_RATE) {
                humanPlayerComPeriodic(true); // Turn lights off
            } else {
                humanPlayerComPeriodic(false); // Turn lights on
            }

            if (Timer.getFPGATimestamp() - humanPlayerBlinkLastTime > HUMAN_PLAYER_COM_BLINK_RATE * 2) {
                humanPlayerBlinkLastTime = Timer.getFPGATimestamp(); // Reset time
            }
        }

        if (currentState == LEDState.PreMatch) {

            if (NetworkTableInstance.getDefault().isConnected()) {
              

                if (false) {
                    // Red team

                    candle.animate(new SingleFadeAnimation(255, 0, 0, 0, 0.1, TOTAL_STRIP_LENGTH, OFFSET_LENGTH), 1);

                } else {
                    // Blue Team
                    candle.animate(new SingleFadeAnimation(0, 0, 255, 0, 0.1, TOTAL_STRIP_LENGTH, OFFSET_LENGTH), 1);

                }
            } else {

                candle.animate(new SingleFadeAnimation(255, 0, 255, 0, 0.1, TOTAL_STRIP_LENGTH, OFFSET_LENGTH), 1);
            }
        }

    }

    public double getVbat() {
        return candle.getBusVoltage();
    }

    public double get5V() {
        return candle.get5VRailVoltage();
    }

    public double getCurrent() {
        return candle.getCurrent();
    }

    public double getTemperature() {
        return candle.getTemperature();
    }

    public void configBrightness(double percent) {
        candle.configBrightnessScalar(percent, 0);
    }

    public void configLos(boolean disableWhenLos) {
        candle.configLOSBehavior(disableWhenLos, 0);
    }

    public void configLedType(LEDStripType type) {
        candle.configLEDType(type, 0);
    }

    public void configStatusLedBehavior(boolean offWhenActive) {
        candle.configStatusLedState(offWhenActive, 0);
    }

    public void setAllToColor(int r, int g, int b) {

        candle.setLEDs(r, g, b, 255, 0, TOTAL_STRIP_LENGTH);

    }

    /**
     * @return The Single instance of Singleton LimeLight
     */
    public static Candle getInstance() {
        // To ensure only one instance is created
        if (CANDLE == null) {
            CANDLE = new Candle();
        }
        return CANDLE;
    }

}
