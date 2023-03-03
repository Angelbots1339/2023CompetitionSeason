// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team254.util.TalonFXFactory;
import frc.lib.team254.util.TalonUtil;
import frc.lib.util.multiplexer.ColorSensorMUXed;
import frc.lib.util.multiplexer.DistanceSensorMUXed;
import frc.robot.Constants;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {

  private Color lastColor = new Color(0, 0, 0);
  private double lastLeftDist = 0;
  private double lastRightDist = 0;

  private final TalonFX intakeMotor = TalonFXFactory.createDefaultTalon(INTAKE_MOTOR_ID, Constants.CANIVORE);;

  private ColorSensorMUXed cubeSensor = new ColorSensorMUXed(0);
  private DistanceSensorMUXed rightConeSensor = new DistanceSensorMUXed(2, RangeProfile.kHighAccuracy);
  private DistanceSensorMUXed leftConeSensor = new DistanceSensorMUXed(3, RangeProfile.kHighAccuracy);

  private double currentIntakePercent = 0;

  /** Creates a new IntakeAndShooter. */
  public Intake() {
    configFalcons();
  }

  @Override
  public void periodic() {
    updateSensors();

    SmartDashboard.putNumber("Cone offset", getConeOffset());
    SmartDashboard.putNumber("Cone left", lastLeftDist);
    SmartDashboard.putNumber("Cone right", lastRightDist);
    SmartDashboard.putBoolean("objectInIntake", objectInIntake());
  }

  int i = 0;

  public void updateSensors() {
    switch (i) {
      case 0:
      lastColor = cubeSensor.get().getColor();
      i++;
        break;
      case 1:
      lastLeftDist = leftConeSensor.getRange();
      i++;
        break;
      case 2:
      lastRightDist = rightConeSensor.getRange();
      i = 0;
        break;
    }
    
  }

  public void disable() {
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getConeOffset() {
    return lastLeftDist - lastRightDist;
  }

  public boolean coneInIntake() {
    return objectInIntake() && ColorSensorMUXed.xyzColorDifference(lastColor, CONE_COLOR) < ColorSensorMUXed
        .xyzColorDifference(lastColor, CUBE_COLOR);
  }

  public void setCurrentIntakePercent(double percent) {
    currentIntakePercent = percent;
  }

  public void runIntake() {
    intakeMotor.set(ControlMode.PercentOutput, currentIntakePercent);
  }

  public void resetSensors() {

    cubeSensor = new ColorSensorMUXed(0);
    rightConeSensor = new DistanceSensorMUXed(2, RangeProfile.kHighAccuracy);
    leftConeSensor = new DistanceSensorMUXed(3, RangeProfile.kHighAccuracy);
  }

  public boolean leftEmpty() {
    return lastLeftDist < 0;
  }

  public boolean rightEmpty() {
    return lastRightDist < 0;
  }

  public boolean objectInIntake() {
    return lastLeftDist > DISTANCE_SENSOR_EMPTY_THRESHOLD
        && lastRightDist > DISTANCE_SENSOR_EMPTY_THRESHOLD;
  }

  public boolean cubeInIntake() {
    return objectInIntake() && ColorSensorMUXed.xyzColorDifference(lastColor, CUBE_COLOR) < ColorSensorMUXed
        .xyzColorDifference(lastColor, CONE_COLOR);
  }

  public void runIntakeAtPercent(double conePercent) {
    intakeMotor.set(ControlMode.PercentOutput, conePercent);
  }

  public IntakeState getState() {
    if (cubeInIntake()) {
      return IntakeState.CUBE;
    } else if (coneInIntake()) {
      return IntakeState.CONE;
    } else {
      return IntakeState.NONE;
    }
  }

  // public void runShootAtPercent(double percent) {
  // shootMotor.set(ControlMode.PercentOutput, percent);
  // }

  private void configFalcons() {
    TalonUtil.checkError(intakeMotor.configVoltageCompSaturation(Constants.MAX_VOLTAGE, Constants.CAN_TIMEOUT),
        "Failed to config Intake Motor Voltage Compensation");
    TalonUtil.checkError(intakeMotor.configSupplyCurrentLimit(CURRENT_LIMIT, Constants.CAN_TIMEOUT),
        "Failed to config Intake Motor Current Limit");
    intakeMotor.selectProfileSlot(0, 0);
    intakeMotor.setInverted(INTAKE_MOTOR_INVERTED);
    intakeMotor.enableVoltageCompensation(true);
  }

  public enum IntakeState {
    CUBE, CONE, NONE
  }
}
