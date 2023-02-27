// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team254.util.TalonFXFactory;
import frc.lib.team254.util.TalonUtil;
import frc.lib.util.multiplexer.ColorSensorMUXed;
import frc.lib.util.multiplexer.DistanceSensorMUXed;
import frc.robot.Constants;

import static frc.robot.Constants.IntakeConstants.*;

public class Intake extends SubsystemBase {

  private final TalonFX intakeMotor = TalonFXFactory.createDefaultTalon(INTAKE_MOTOR_ID, Constants.CANIVORE);;
 
  private final ColorSensorMUXed cubeSensor = new ColorSensorMUXed(0);
  private final DistanceSensorMUXed rightConeSensor = new DistanceSensorMUXed(2, RangeProfile.kHighAccuracy);
  private final DistanceSensorMUXed leftConeSensor = new DistanceSensorMUXed(1, RangeProfile.kHighAccuracy);

  private double currentIntakePercent = 0;


  /** Creates a new IntakeAndShooter. */
  public Intake() {
    configFalcons();
  }

  @Override
  public void periodic() {

    SmartDashboard.putNumber("offset", getConeOffset());
  }


  public void disable(){
    intakeMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getConeOffset(){
    return leftConeSensor.getRange() - rightConeSensor.getRange();
  }

  public boolean coneInIntake(){
    return objectInIntake() && cubeSensor.xyzColorDifference(CONE_COLOR) < cubeSensor.xyzColorDifference(CUBE_COLOR);
  }

  public void setCurrentIntakePercent(double percent){
    currentIntakePercent = percent;
  }

  public void runIntake(){
    intakeMotor.set(ControlMode.PercentOutput, currentIntakePercent);
  }
  


  public boolean objectInIntake(){
    return leftConeSensor.getRange() > DISTANCE_SENSOR_EMPTY_THRESHOLD && rightConeSensor.getRange() > DISTANCE_SENSOR_EMPTY_THRESHOLD;
  }

  public boolean cubeInIntake(){
    return objectInIntake() && cubeSensor.xyzColorDifference(CUBE_COLOR) < cubeSensor.xyzColorDifference(CONE_COLOR);
  }
  public void runIntakeAtPercent(double conePercent) {
    intakeMotor.set(ControlMode.PercentOutput, conePercent);
  }
  
  public IntakeState getState(){
    if(cubeInIntake()){
      return IntakeState.CUBE;
    } else if(coneInIntake()){
      return IntakeState.CONE;
    } else {
      return IntakeState.NONE;
    }
  }

  // public void runShootAtPercent(double percent) {
  //   shootMotor.set(ControlMode.PercentOutput, percent);
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
