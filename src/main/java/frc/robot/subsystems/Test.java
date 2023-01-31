// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Test extends SubsystemBase {

  private TalonFX test = new TalonFX(1, Constants.CANIVORE);
  private Pigeon2 pigeon2 = new Pigeon2(1);
  /** Creates a new Test. */
  public Test() {

  }

  @Override
  public void periodic() {


    SmartDashboard.putNumber("Test", test.getSelectedSensorVelocity());
    // This method will be called once per scheduler run
  }
}
