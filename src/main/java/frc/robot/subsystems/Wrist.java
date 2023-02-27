// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team254.util.TalonFXFactory;
import frc.lib.team254.util.TalonUtil;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorWristStateConstants;

import static frc.robot.Constants.WristConstants.*;

public class Wrist extends SubsystemBase {

  private final TalonFX wristMotor;
  private final DutyCycleEncoder dutyCycleEncoder = new DutyCycleEncoder(23);
  private final Timer throughBoreTimer = new Timer();
  private Rotation2d goalAngle = Rotation2d.fromDegrees(0);
  /** Creates a new Wrist. */
  public Wrist() {
    wristMotor =  TalonFXFactory.createDefaultTalon(MOTOR_ID, Constants.CANIVORE);
    configTalons();
    throughBoreTimer.start();
  }

  public void setPercent(double percent){
    double cosineScaler = Math.cos(getAngleFromHorizontal().getRadians());
    wristMotor.set(ControlMode.PercentOutput, percent, DemandType.ArbitraryFeedForward, cosineScaler * KS);
  }
  public void setMotionMagic(double clicks){
    goalAngle = new Rotation2d(clicksToRadians(clicks));

    double cosineScaler = Math.cos(getAngleFromHorizontal().getRadians());
    if(clicks == radiansToClicks(ElevatorWristStateConstants.HOME.angle.getRadians()) && Math.abs(getAngleDeg() - ElevatorWristStateConstants.HOME.angle.getDegrees()) <= 1){
      SmartDashboard.putNumber("error", wristMotor.getClosedLoopError());
      wristMotor.set(ControlMode.PercentOutput, 0);
    }
    else
    wristMotor.set(ControlMode.MotionMagic, clicks, DemandType.ArbitraryFeedForward, cosineScaler * KS);
  }
  public double getErrorDeg(){
    return Math.abs(getAngleDeg() - goalAngle.getDegrees());
  }
  public boolean atSetPoint(){
    return getErrorDeg() < MOTION_MAGIC_ERROR_THRESHOLD;
  }
  public boolean atSetPointAndSettled(){
    return settleTimer.hasElapsed(TIME_TO_SETTLE);
  }
 
  private final Timer settleTimer = new Timer();
  private void updateSetPointWatcher(){
    if(atSetPoint())
      settleTimer.start();
    else if (settleTimer.get() != 0){
      settleTimer.stop();
      settleTimer.restart();
    }
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
   * @return the angle of the wrist in radians from start point or resting position
   */
  public double getAngleDeg(){
    return getAngle().getDegrees();
  }
  /**
   * @return the angle of the wrist in radians from horizontal 
   * positive angle above horizontal and a negative angle below horizontal.
   */
  public Rotation2d getAngleFromHorizontal(){
    return new Rotation2d(clicksToRadians(CLICKS_AT_HORIZONTAL - wristMotor.getSelectedSensorPosition()));
  }

  public boolean goalAtHome() {
    return goalAngle.getDegrees() == ElevatorWristStateConstants.HOME.angle.getDegrees();
  }

  public double getVelocityClicks(){
    return wristMotor.getSelectedSensorVelocity();

  }

  public void disable(){
    wristMotor.set(ControlMode.PercentOutput, 0);
  }

  public void resetToAbsolute(){
   double absolutePosition = Math.toRadians(angleOffset - throughBoreToAngle(dutyCycleEncoder.getAbsolutePosition()).getDegrees());
   wristMotor.setSelectedSensorPosition(radiansToClicks(absolutePosition));
  }







  @Override
  public void periodic() {
    if(throughBoreTimer.get() >= TimeBeforeReset){
      resetToAbsolute();
      throughBoreTimer.reset();
      throughBoreTimer.stop();
    }

    updateSetPointWatcher();

    SmartDashboard.putNumber("angle", getAngleDeg());
    SmartDashboard.putNumber("through bore", throughBoreToAngle(dutyCycleEncoder.get()).getDegrees());
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
    TalonUtil.checkError(wristMotor.configStatorCurrentLimit(STATOR_CURRENT_LIMIT, Constants.CAN_TIMEOUT),
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
