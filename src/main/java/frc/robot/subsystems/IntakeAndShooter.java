// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team254.util.TalonFXFactory;
import frc.lib.team254.util.TalonUtil;
import frc.lib.util.multiplexer.DistanceSensorMUXed;
import frc.robot.Constants;
import frc.robot.Robot;

import static frc.robot.Constants.IntakeConstants.*;

public class IntakeAndShooter extends SubsystemBase {

  TalonFX intakeMotor = TalonFXFactory.createDefaultTalon(INTAKE_MOTOR_ID, Constants.CANIVORE);;
  TalonFX shootMotor = TalonFXFactory.createDefaultTalon(SHOOT_MOTOR_ID, Constants.CANIVORE);;

  //DistanceSensorMUXed uprightConeSensor;
  //DistanceSensorMUXed fallenConeSensor;
  //DistanceSensorMUXed cubeSensor;

  /** Creates a new IntakeAndShooter. */
  public IntakeAndShooter() {
    configFalcons();
    if (Robot.isReal()) {
      //cubeSensor = new DistanceSensorMUXed(CUBE, DISTANCE_SENSOR_PROFILE);
      //uprightConeSensor = new DistanceSensorMUXed(UPRIGHT_CONE, DISTANCE_SENSOR_PROFILE);
      //fallenConeSensor = new DistanceSensorMUXed(FALLEN_CONE, DISTANCE_SENSOR_PROFILE);

    }
  }

  @Override
  public void periodic() {

    if (Robot.isReal()) {
      //SmartDashboard.putNumber("cube", cubeSensor.getRange());
      //SmartDashboard.putNumber("upright cone", uprightConeSensor.getRange());
      //SmartDashboard.putNumber("fallen cone", fallenConeSensor.getRange());
    }

    // This method will be called once per scheduler run
  }

  public boolean UprightConeInRange() {
    //return uprightConeSensor.getRange() >= CONE_SENSOR_THRESHOLD;
    return false;
  }

  public boolean FallenConeInRange() {
    // return fallenConeSensor.getRange() >= CONE_SENSOR_THRESHOLD;
    return false;
  }

  public boolean CubeInRange() {
    // return cubeSensor.getRange() >= CUBE_SENSOR_THRESHOLD;
    return false;
  }

  public void runConeIntakeAtCurrent(double amps) {
    intakeMotor.set(ControlMode.Current, amps);
  }

  public void runCubeIntakeAtCurrent(double amps) {
    intakeMotor.set(ControlMode.Current, amps);
    shootMotor.set(ControlMode.Current, amps);
  }

  public void runConeIntakeAtPercent(double percent) {
    shootMotor.set(ControlMode.PercentOutput, -percent);
    intakeMotor.set(ControlMode.PercentOutput, percent);
  }
  public void runCubeIntakeAtPercent(double percent) {
    shootMotor.set(ControlMode.PercentOutput, percent);
    intakeMotor.set(ControlMode.PercentOutput, -percent);
  }

  public void runShootAtPercent(double percent) {
    shootMotor.set(ControlMode.PercentOutput, percent);
  }

  private void configFalcons() {
    TalonUtil.checkError(intakeMotor.configVoltageCompSaturation(Constants.MAX_VOLTAGE, Constants.CAN_TIMEOUT),
        "Failed to config Intake Motor Voltage Compensation");
    TalonUtil.checkError(intakeMotor.configSupplyCurrentLimit(CURRENT_LIMIT, Constants.CAN_TIMEOUT),
        "Failed to config Intake Motor Current Limit");
    TalonUtil.checkError(intakeMotor.config_kP(0, KP, Constants.CAN_TIMEOUT), "Failed to config Intake Motor KP");
    TalonUtil.checkError(intakeMotor.config_kD(0, KD, Constants.CAN_TIMEOUT), "Failed to config Intake Motor KD");
    TalonUtil.checkError(intakeMotor.config_kF(0, KF, Constants.CAN_TIMEOUT), "Failed to config Intake Motor KF");

    intakeMotor.selectProfileSlot(0, 0);
    intakeMotor.setInverted(INTAKE_MOTOR_INVERTED);
    intakeMotor.enableVoltageCompensation(true);

    TalonUtil.checkError(shootMotor.configVoltageCompSaturation(Constants.MAX_VOLTAGE, Constants.CAN_TIMEOUT),
        "Failed to config Shoot Motor Voltage Compensation");
    TalonUtil.checkError(shootMotor.configSupplyCurrentLimit(CURRENT_LIMIT, Constants.CAN_TIMEOUT),
        "Failed to config Shoot Motor Current Limit");
    TalonUtil.checkError(shootMotor.config_kP(0, KP, Constants.CAN_TIMEOUT), "Failed to config Shoot Motor KP");
    TalonUtil.checkError(shootMotor.config_kD(0, KD, Constants.CAN_TIMEOUT), "Failed to config Shoot Motor KD");
    TalonUtil.checkError(shootMotor.config_kF(0, KF, Constants.CAN_TIMEOUT), "Failed to config Shoot Motor KF");

    shootMotor.selectProfileSlot(0, 0);
    shootMotor.setInverted(SHOOT_MOTOR_INVERTED);
    shootMotor.enableVoltageCompensation(true);
  }
}
