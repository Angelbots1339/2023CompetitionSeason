// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team254.util.TalonFXFactory;
import frc.robot.Constants;

import static frc.robot.Constants.ElevatorConstants.*;

public class Elevator extends SubsystemBase {

  TalonFX elevatorLeaderMotor;
  TalonFX elevatorFollowerMotor;

  /** Creates a new Elevator. */
  public Elevator() {
    elevatorLeaderMotor = TalonFXFactory.createDefaultTalon(ELEVATOR_LEADER_MOTOR_ID, Constants.CANIVORE);
    elevatorFollowerMotor = TalonFXFactory.createPermanentFollowerTalon(ELEVATOR_FOLLOWER_MOTOR_ID, ELEVATOR_LEADER_MOTOR_ID, Constants.CANIVORE);
    configElevatorMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void configElevatorMotor(){

  }

}
