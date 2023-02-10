// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.lib.team254.util.TalonFXFactory;
import frc.lib.team254.util.TalonUtil;
import frc.lib.util.logging.LoggedSubsystem;
import frc.robot.Constants;
import frc.robot.LoggingConstants;

import static frc.robot.Constants.ElevatorConstants.*;

public class Elevator extends SubsystemBase{

  TalonFX elevatorLeaderMotor;
  TalonFX elevatorFollowerMotor;

  TalonFXSimCollection elevatorLeaderMotorSim;
  TalonFXSimCollection elevatorFollowerMotorSim;
  
  private Mechanism2d mech = new Mechanism2d(4, 4);

  private MechanismLigament2d elevator;


 




  LoggedSubsystem logger;

  /** Creates a new Elevator. */
  public Elevator() {
    //elevatorLeaderMotor = TalonFXFactory.createDefaultTalon(ELEVATOR_LEADER_MOTOR_ID, Constants.CANIVORE);
    elevatorLeaderMotor = new WPI_TalonFX(ELEVATOR_LEADER_MOTOR_ID, Constants.CANIVORE);
    elevatorLeaderMotorSim = elevatorLeaderMotor.getSimCollection();

    // elevatorFollowerMotor = TalonFXFactory.createPermanentFollowerTalon(ELEVATOR_FOLLOWER_MOTOR_ID,
    //     ELEVATOR_LEADER_MOTOR_ID, Constants.CANIVORE);

    elevatorFollowerMotor = new WPI_TalonFX(ELEVATOR_FOLLOWER_MOTOR_ID, Constants.CANIVORE);
    elevatorFollowerMotor.set(ControlMode.Follower, ELEVATOR_LEADER_MOTOR_ID);
    elevatorFollowerMotorSim = elevatorFollowerMotor.getSimCollection();
  
    configElevatorMotor();


    logger = new LoggedSubsystem("Elevator", LoggingConstants.ELEVATOR);


    MechanismRoot2d root = mech.getRoot("climber", 2, 0);

    elevator = root.append(new MechanismLigament2d("elevator", 2, 90));


    

  }

    

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();

    elevator.setLength(convertTicksToMeters(elevatorLeaderMotor.getSelectedSensorPosition()));


  }

  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }

  public void setPercent(double percent){
    elevatorLeaderMotor.set(ControlMode.PercentOutput, percent);

    SmartDashboard.putNumber("jpu ", percent);

  }

  /**
   * Converts the extension of the elevator relative to the home position in meters
   * @param meters
   * @return
   */
  public static double convertExtensionsMetersToTicks(double meters) {
    double rotations = meters / 2 / (Math.PI * ELEVATOR_SPOOL_DIAMETER);
    return Conversions.RotationsToFalcons(rotations, ELEVATOR_GEAR_RATIO);
  }

  public static double convertTicksToMeters(double ticks) {
    double rotations = Conversions.falconToRotations(ticks, ELEVATOR_GEAR_RATIO);
    return rotations * 2 * Math.PI * ELEVATOR_SPOOL_DIAMETER;
  }


 /**
   * Converts the extension of the elevator relative to the home position in meters
   * @param meters
   * @return
   */
  public static double mPSToCPS(double MetersPerSec) {
    return convertExtensionsMetersToTicks(MetersPerSec) / 10;//to convert to 100ms loops
  }

  public static double cPSToMPS(double CountsPerSec) {
    return convertTicksToMeters(CountsPerSec * 10);//to convert to 100ms loops
  }



  public void setElevatorMotorExtension(double meters) {
    elevatorLeaderMotor.set(TalonFXControlMode.MotionMagic, convertExtensionsMetersToTicks(meters), DemandType.ArbitraryFeedForward, ELEVATOR_KS);
  }

  private void configElevatorMotor() {
    TalonUtil.checkError(elevatorLeaderMotor.configVoltageCompSaturation(Constants.MAX_VOLTAGE, Constants.CAN_TIMEOUT),
        "Failed to set voltage compensation saturation elevator motor"); 
        TalonUtil.checkError(
        elevatorLeaderMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
            LimitSwitchNormal.NormallyClosed, Constants.CAN_TIMEOUT),
        "Failed to set forward limit switch source elevator motor");
    TalonUtil.checkError(
        elevatorLeaderMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
            LimitSwitchNormal.NormallyClosed, Constants.CAN_TIMEOUT),
        "Failed to set reverse limit switch source elevator motor");
    TalonUtil.checkError(
        elevatorLeaderMotor.configForwardSoftLimitThreshold(ELEVATOR_FORWARD_SOFT_LIMIT, Constants.CAN_TIMEOUT),
        "Failed to set forward soft limit threshold elevator motor");
    TalonUtil.checkError(
        elevatorLeaderMotor.configReverseSoftLimitThreshold(ELEVATOR_REVERSE_SOFT_LIMIT, Constants.CAN_TIMEOUT),
        "Failed to set reverse soft limit threshold elevator motor");
    TalonUtil.checkError(elevatorLeaderMotor.configForwardSoftLimitEnable(true, Constants.CAN_TIMEOUT),
        "Failed to enable forward soft limit elevator motor");
    TalonUtil.checkError(elevatorLeaderMotor.configReverseSoftLimitEnable(true, Constants.CAN_TIMEOUT),
        "Failed to enable reverse soft limit elevator motor");
    TalonUtil.checkError(elevatorLeaderMotor.config_kP(0, ELEVATOR_KP, Constants.CAN_TIMEOUT),
        "Failed to set elevator motor kP");
    TalonUtil.checkError(elevatorLeaderMotor.config_kD(0, ELEVATOR_KD, Constants.CAN_TIMEOUT),
        "Failed to set elevator motor kD");
    TalonUtil.checkError(elevatorLeaderMotor.config_kF(0, ELEVATOR_KF, Constants.CAN_TIMEOUT),
        "Failed to set elevator motor kF");
    TalonUtil.checkError(elevatorLeaderMotor.configSupplyCurrentLimit(ELEVATOR_CURRENT_LIMIT, Constants.CAN_TIMEOUT),
        "Failed to set elevator motor current limit");
    TalonUtil.checkError(elevatorLeaderMotor.configMotionAcceleration(ELEVATOR_MAX_ACCELERATION, Constants.CAN_TIMEOUT),
        "Failed to set elevator motor acceleration");
    TalonUtil.checkError(elevatorLeaderMotor.configMotionSCurveStrength(ELEVATOR_S_CURVE_STRENGTH, Constants.CAN_TIMEOUT),
        "Failed to set elevator motor s curve strength");

    TalonUtil.checkError(elevatorLeaderMotor.configMotionCruiseVelocity(ELEVATOR_MAX_VELOCITY, Constants.CAN_TIMEOUT),
        "Failed to set elevator motor cruise velocity");
    TalonUtil.checkError(
        elevatorFollowerMotor.configAllowableClosedloopError(0, ALLOWABLE_ERROR, Constants.CAN_TIMEOUT),
        "Failed to set elevator follower motor allowable error");
    
    TalonUtil.checkError(elevatorLeaderMotor.configClearPositionOnLimitR(true, Constants.CAN_TIMEOUT),
        "Failed to set elevator leader motor clear position on limit R");

    TalonUtil.checkError(elevatorLeaderMotor.configVelocityMeasurementWindow(ELEVATOR_SENSOR_VELOCITY_MEAS_WINDOW, Constants.CAN_TIMEOUT),
        "Failed to set elevator leader motor velocity measurement window");
    
    elevatorLeaderMotor.enableVoltageCompensation(true);

    elevatorLeaderMotor.setNeutralMode(ELEVATOR_NEUTRAL_MODE);

    elevatorLeaderMotor.selectProfileSlot(0, 0);

    elevatorLeaderMotor.setInverted(LEADER_MOTOR_INVERTED);
    elevatorLeaderMotor.setInverted(TalonFXInvertType.OpposeMaster);

  }

  private void initializeLog(){
  }

  

}
