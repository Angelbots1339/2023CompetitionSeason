// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team254.util.TalonFXFactory;
import frc.lib.team254.util.TalonUtil;
import frc.lib.util.Candle;
import frc.lib.util.logging.LoggedSubsystem;
import frc.lib.util.logging.loggedObjects.LoggedFalcon;
import frc.lib.util.multiplexer.ColorSensorMUXed;
import frc.lib.util.multiplexer.DistanceSensorMUXed;
import frc.lib.util.multiplexer.Multiplexer;
import frc.robot.Constants;
import frc.robot.FieldDependentConstants;
import frc.robot.LoggingConstants;
import frc.robot.regressions.ConeOffsetRegression;

import static frc.robot.Constants.IntakeConstants.*;

import java.util.Optional;

public class Intake extends SubsystemBase {


  private final LoggedSubsystem logger;


  private final TalonFX intakeMotor = TalonFXFactory.createDefaultTalon(INTAKE_MOTOR_ID, Constants.CANIVORE);
  private Rev2mDistanceSensor rightConeSensor = new Rev2mDistanceSensor(Port.kMXP);

  private double distSensorOverride = 0;

  private double currentIntakePercent = 0;

  private IntakeState deadSensorOverrideSate = IntakeState.CONE;
  private String command = "None";

  /** Creates a new IntakeAndShooter. */
  public Intake() {
    configFalcons();
    rightConeSensor.setRangeProfile(RangeProfile.kHighAccuracy);
    rightConeSensor.setAutomaticMode(true);
    rightConeSensor.setEnabled(true);
    rightConeSensor.setDistanceUnits(Unit.kMillimeters);
    rightConeSensor.setMeasurementPeriod(1);
    logger = new LoggedSubsystem("Intake", LoggingConstants.INTAKE);



    logger.addDouble("RightConeSensor", () -> rightConeSensor.getRange(), "Main");
    logger.addBoolean("SensorDead", () -> getSensorDead(), "Main");
    logger.addString("Command", () -> {
      Optional.ofNullable(this.getCurrentCommand()).ifPresent((Command c) -> {command = c.getName();});
      return command;
    }, "Main");


    logger.add(new LoggedFalcon("IntakeMotor", logger, intakeMotor, "Motor"));
  }

 


  @Override
  public void periodic() {
    SmartDashboard.putNumber("ConeOffset", getConeOffset());
    SmartDashboard.putBoolean("SensorDead", getSensorDead());
    // SmartDashboard.putNumber("ConeSensor", rightConeSensor.getRange() / 1000);

    SmartDashboard.putBoolean("objectInIntake", objectInIntake());
    SmartDashboard.putString("override State", deadSensorOverrideSate.toString());

    String align = "mid";
    if (getConeOffset() > FieldDependentConstants.CurrentField.LIMELIGHT_ALIGN_LEFT_BOUND) {
      align = "coneLeft";
    } else if (getConeOffset() < FieldDependentConstants.CurrentField.LIMELIGHT_ALIGN_RIGHT_BOUND) {
      align = "coneRight";
    }

    SmartDashboard.putString("ConeLocation", align);

  }

 
  public void disable() {
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public void resetDistSensorOverride(){
    distSensorOverride = 0;
  }
  public void setDistSensorOverrideLeft(){
    distSensorOverride = FieldDependentConstants.CurrentField.CONE_BACKUP_OFFSET;
  }
  public void setDistSensorOverrideRight(){
    distSensorOverride = -FieldDependentConstants.CurrentField.CONE_BACKUP_OFFSET;
  }


  public double getConeOffset() {
    if(getSensorDead()){
      return distSensorOverride;
    }
    return (rightConeSensor.GetRange() / 1000) - FieldDependentConstants.CurrentField.CONE_OFFSET;
  }
  public boolean getSensorDead() {
    return rightConeSensor.getRange() < 0;
  }


  public void setCurrentIntakePercent(double percent) {
    currentIntakePercent = percent;
  }

  public void runIntake() {
    intakeMotor.set(ControlMode.PercentOutput, currentIntakePercent);
  }

  public void resetSensors() {
    rightConeSensor = new Rev2mDistanceSensor(Port.kMXP);
    rightConeSensor.setRangeProfile(RangeProfile.kHighAccuracy);
    rightConeSensor.setAutomaticMode(true);
    rightConeSensor.setEnabled(true);
    rightConeSensor.setDistanceUnits(Unit.kMillimeters);
  }

  
  public boolean objectInIntake() {
    return (rightConeSensor.GetRange() / 1000) > 0 && (rightConeSensor.GetRange()/1000) < 0.28;
  }

  public void runIntakeAtPercent(double conePercent) {
    intakeMotor.set(ControlMode.PercentOutput, conePercent);
  }

  public IntakeState getState() {
    return deadSensorOverrideSate;
  }

  public IntakeState getDeadSensorOverrideSate() {
      return deadSensorOverrideSate;
  }

  public void setDeadSensorOverrideSate(IntakeState deadSensorOverrideSate) {
      this.deadSensorOverrideSate = deadSensorOverrideSate;
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
