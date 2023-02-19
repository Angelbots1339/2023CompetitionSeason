// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team254.util.TalonFXFactory;
import frc.lib.team254.util.TalonUtil;
import frc.robot.Constants;
import frc.robot.Robot;

import static frc.robot.Constants.WristConstants.*;

public class Wrist extends SubsystemBase {

  TalonFX wristMotor;
  TalonFXSimCollection wristMotorSim;
  /** Creates a new Wrist. */
  public Wrist() {
    if(Robot.isReal()){
      wristMotor =  TalonFXFactory.createDefaultTalon(MOTOR_ID, Constants.CANIVORE);
    }
    else if(Robot.isSimulation()){
      wristMotor = TalonFXFactory.createDefaultSimulationTalon(MOTOR_ID, Constants.CANIVORE);
      wristMotorSim = wristMotor.getSimCollection();
    }
    configTalons();
  }

  public void setPercent(double percent){
    double cosineScaler = Math.cos(getAngleFromHorizontal().getRadians());
    wristMotor.set(ControlMode.PercentOutput, percent, DemandType.ArbitraryFeedForward, cosineScaler * KS);
  }
  public void setMotionMagic(double clicks){
    double cosineScaler = Math.cos(getAngleFromHorizontal().getRadians());
    wristMotor.set(ControlMode.MotionMagic, clicks, DemandType.ArbitraryFeedForward, cosineScaler * KS);
  }
  public void setPid(double clicks){
    double cosineScaler = Math.cos(getAngleFromHorizontal().getRadians());

 
    wristMotor.set(ControlMode.Position, clicks, DemandType.ArbitraryFeedForward, cosineScaler * KS);
  }



  /**
   * @param position in falcon clicks
   */
  public void resetToPosition(double position){
    wristMotor.setSelectedSensorPosition(0);
  }


  /**
   * @return the angle of the wrist in radians from start point or resting position
   */
  public Rotation2d getAngle(){
    return new Rotation2d(clicksToRadians(wristMotor.getSelectedSensorPosition()));
  }
  /**
   * @return the angle of the wrist in radians from horizontal 
   * positive angle above horizontal and a negative angle below horizontal.
   */
  public Rotation2d getAngleFromHorizontal(){
    return new Rotation2d(clicksToRadians(CLICKS_AT_HORIZONTAL - wristMotor.getSelectedSensorPosition()));
  }

  public double getVelocityClicks(){
    return wristMotor.getSelectedSensorVelocity();

  }

  public void disable(){
    wristMotor.set(ControlMode.PercentOutput, 0);

  }



  @Override
  public void periodic() {
    SmartDashboard.putNumber("Velocity", getVelocityClicks());
    SmartDashboard.putNumber("Positon", getAngle().getDegrees());
    
    double cosineScaler = Math.cos(getAngleFromHorizontal().getRadians());
    SmartDashboard.putNumber("Angle", getAngleFromHorizontal().getDegrees());
    SmartDashboard.putNumber("ks", cosineScaler * KS);
    
    // This method will be called once per scheduler run
  }



  public void configTalons(){
    TalonUtil.checkError(wristMotor.configVoltageCompSaturation(Constants.MAX_VOLTAGE, Constants.CAN_TIMEOUT),
        "Failed to set voltage compensation saturation wrist motor");
    TalonUtil.checkError(
      wristMotor.configForwardSoftLimitThreshold(FORWARD_SOFT_LIMIT, Constants.CAN_TIMEOUT),
        "Failed to set forward soft limit threshold wrist motor");
    TalonUtil.checkError(
      wristMotor.configReverseSoftLimitThreshold(REVERSE_SOFT_LIMIT, Constants.CAN_TIMEOUT),
        "Failed to set reverse soft limit threshold wrist motor");
    TalonUtil.checkError(wristMotor.configForwardSoftLimitEnable(true, Constants.CAN_TIMEOUT),
        "Failed to enable forward soft limit wrist motor");
    TalonUtil.checkError(wristMotor.configReverseSoftLimitEnable(true, Constants.CAN_TIMEOUT),
        "Failed to enable reverse soft limit wrist motor");
    TalonUtil.checkError(wristMotor.config_kP(0, KP, Constants.CAN_TIMEOUT),
        "Failed to set wrist motor kP");
    TalonUtil.checkError(wristMotor.config_kD(0, KD, Constants.CAN_TIMEOUT),
        "Failed to set wrist motor kD");
    TalonUtil.checkError(wristMotor.config_kF(0, KF, Constants.CAN_TIMEOUT),
        "Failed to set wrist motor kF");
    TalonUtil.checkError(wristMotor.configSupplyCurrentLimit(CURRENT_LIMIT, Constants.CAN_TIMEOUT),
        "Failed to set wrist motor current limit");
    TalonUtil.checkError(wristMotor.configMotionAcceleration(MAX_ACCELERATION, Constants.CAN_TIMEOUT),
        "Failed to set wrist motor acceleration");
    TalonUtil.checkError(
        wristMotor.configMotionSCurveStrength(S_CURVE_STRENGTH, Constants.CAN_TIMEOUT),
        "Failed to set wrist motor s curve strength");
    TalonUtil.checkError(wristMotor.configMotionCruiseVelocity(MAX_VELOCITY, Constants.CAN_TIMEOUT),
        "Failed to set wrist motor cruise velocity");
    TalonUtil.checkError(wristMotor.configClearPositionOnLimitR(true, Constants.CAN_TIMEOUT),
        "Failed to set wrist leader motor clear position on limit R");
    TalonUtil.checkError(
        wristMotor.configVelocityMeasurementWindow(SENSOR_VELOCITY_MEAS_WINDOW,
            Constants.CAN_TIMEOUT),
        "Failed to set wrist leader motor velocity measurement window");
        
    wristMotor.enableVoltageCompensation(true);
    wristMotor.overrideSoftLimitsEnable(true);
    wristMotor.setNeutralMode(NEUTRAL_MODE);
    wristMotor.selectProfileSlot(0, 0);
    wristMotor.setInverted(MOTOR_INVERTED);

  }
}
