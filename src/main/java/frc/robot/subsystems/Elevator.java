// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlFrame;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.unmanaged.Unmanaged;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.Mech2dManger;
import frc.lib.math.Conversions;
import frc.lib.team254.util.SimulationUtils;
import frc.lib.team254.util.TalonFXFactory;
import frc.lib.team254.util.TalonUtil;
import frc.lib.util.logging.LoggedSubsystem;
import frc.robot.Constants;
import frc.robot.LoggingConstants;
import frc.robot.Robot;

import static frc.robot.Constants.ElevatorConstants.*;

public class Elevator extends SubsystemBase {

  TalonFX elevatorLeaderMotor;
  TalonFX elevatorFollowerMotor;

  TalonFXSimCollection elevatorLeaderMotorSim;


  private MechanismLigament2d elevator;

  private ElevatorSim elevatorSim;
  LoggedSubsystem logger;

  /** Creates a new Elevator. */
  public Elevator() {

    if (Robot.isReal()) {
      elevatorLeaderMotor = TalonFXFactory.createDefaultTalon(LEADER_MOTOR_ID, Constants.CANIVORE);
      TalonFX elevatorFollowerMotor= TalonFXFactory.createPermanentFollowerTalon(FOLLOWER_MOTOR_ID,
          LEADER_MOTOR_ID, Constants.CANIVORE);

        elevatorFollowerMotor.setInverted(TalonFXInvertType.OpposeMaster);
        elevatorFollowerMotor.setNeutralMode(NeutralMode.Brake);





    } else if (Robot.isSimulation()) {
      elevatorLeaderMotor = TalonFXFactory.createDefaultSimulationTalon(LEADER_MOTOR_ID, Constants.CANIVORE);
      elevatorLeaderMotorSim = elevatorLeaderMotor.getSimCollection();

      TalonFX elevatorFollowerMotor = TalonFXFactory.createPermanentSimulationFollowerTalon(FOLLOWER_MOTOR_ID,
          LEADER_MOTOR_ID, Constants.CANIVORE);
      elevatorFollowerMotor.getSimCollection();
      elevatorFollowerMotor.setInverted(TalonFXInvertType.OpposeMaster);
      elevatorSim = new ElevatorSim(DCMotor.getFalcon500(2), GEAR_RATIO, 19.0509,
        SPOOL_DIAMETER, 0, 1.4732, false);
    }

    configElevatorMotor();
    logger = new LoggedSubsystem("Elevator", LoggingConstants.ELEVATOR);


    elevator = Mech2dManger.getInstance().getElevator();
  }

  

  @Override
  public void simulationPeriodic() {
    super.simulationPeriodic();


    
    elevatorSim.setInputVoltage(elevatorLeaderMotorSim.getMotorOutputLeadVoltage());
    elevatorSim.update(0.02);
    elevatorLeaderMotorSim.setLimitFwd(elevatorSim.hasHitUpperLimit());
    elevatorLeaderMotorSim.setLimitRev(elevatorSim.hasHitLowerLimit());
    elevatorLeaderMotorSim.setIntegratedSensorRawPosition(metersToClicks(elevatorSim.getPositionMeters()));
    elevatorLeaderMotorSim.setIntegratedSensorVelocity(mPSToCP100ms(elevatorSim.getVelocityMetersPerSecond()));
    
     elevator.setLength(0.2 + elevatorSim.getPositionMeters());

  }

  @Override
  public void periodic() {

    //SmartDashboard.putNumber("EVelocity", elevatorLeaderMotor.getSelectedSensorVelocity());
    SmartDashboard.putNumber("EVelocity", getPositionMeters());

    SmartDashboard.putNumber("forward", elevatorLeaderMotor.isFwdLimitSwitchClosed());
    SmartDashboard.putNumber("reverse", elevatorLeaderMotor.isFwdLimitSwitchClosed());

    // This method will be called once per scheduler run
  }

  /**
   * @param position in falcon clicks
   */
  public void resetToPosition(double position){
    elevatorLeaderMotor.setSelectedSensorPosition(0);
  }

  public void setPercent(double percent) {
    elevatorLeaderMotor.set(ControlMode.PercentOutput, percent);
  }

  public void setMotionMagicClicks(double position) {
    if(Robot.isReal())
      elevatorLeaderMotor.set(ControlMode.MotionMagic, position, DemandType.ArbitraryFeedForward, KS);
    else
    elevatorLeaderMotor.set(ControlMode.MotionMagic, position);
  }

  public void disable(){
    elevatorLeaderMotor.set(ControlMode.PercentOutput, 0);
  }

  public double getPositionMeters() {
    return clicksToMeters(elevatorLeaderMotor.getSelectedSensorPosition());
  }

  public double getVelocityMetersPerSecond() {
    return cP100msToMPS(elevatorLeaderMotor.getSelectedSensorVelocity());
  }

  public void setMotionMagicMeters(double meters) {
    elevatorLeaderMotor.set(TalonFXControlMode.MotionMagic, metersToClicks(meters),
        DemandType.ArbitraryFeedForward, KS);
  }

  private void configElevatorMotor() {
    TalonUtil.checkError(elevatorLeaderMotor.configVoltageCompSaturation(Constants.MAX_VOLTAGE, Constants.CAN_TIMEOUT),
        "Failed to set voltage compensation saturation elevator motor");
    TalonUtil.checkError(
        elevatorLeaderMotor.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
            LimitSwitchNormal.NormallyOpen, Constants.CAN_TIMEOUT),
        "Failed to set forward limit switch source elevator motor");
    TalonUtil.checkError(
        elevatorLeaderMotor.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector,
            LimitSwitchNormal.NormallyOpen, Constants.CAN_TIMEOUT),
        "Failed to set reverse limit switch source elevator motor");
    TalonUtil.checkError(
        elevatorLeaderMotor.configForwardSoftLimitThreshold(FORWARD_SOFT_LIMIT, Constants.CAN_TIMEOUT),
        "Failed to set forward soft limit threshold elevator motor");
    TalonUtil.checkError(
        elevatorLeaderMotor.configReverseSoftLimitThreshold(REVERSE_SOFT_LIMIT, Constants.CAN_TIMEOUT),
        "Failed to set reverse soft limit threshold elevator motor");
    TalonUtil.checkError(elevatorLeaderMotor.configForwardSoftLimitEnable(true, Constants.CAN_TIMEOUT),
        "Failed to enable forward soft limit elevator motor");
    TalonUtil.checkError(elevatorLeaderMotor.configReverseSoftLimitEnable(true, Constants.CAN_TIMEOUT),
        "Failed to enable reverse soft limit elevator motor");
    TalonUtil.checkError(elevatorLeaderMotor.config_kP(0, KP, Constants.CAN_TIMEOUT),
        "Failed to set elevator motor kP");
    TalonUtil.checkError(elevatorLeaderMotor.config_kD(0, KD, Constants.CAN_TIMEOUT),
        "Failed to set elevator motor kD");
    TalonUtil.checkError(elevatorLeaderMotor.config_kF(0, KF, Constants.CAN_TIMEOUT),
        "Failed to set elevator motor kF");
    TalonUtil.checkError(elevatorLeaderMotor.configSupplyCurrentLimit(CURRENT_LIMIT, Constants.CAN_TIMEOUT),
        "Failed to set elevator motor current limit");
    TalonUtil.checkError(elevatorLeaderMotor.configMotionAcceleration(MAX_ACCELERATION, Constants.CAN_TIMEOUT),
        "Failed to set elevator motor acceleration");
    TalonUtil.checkError(
        elevatorLeaderMotor.configMotionSCurveStrength(S_CURVE_STRENGTH, Constants.CAN_TIMEOUT),
        "Failed to set elevator motor s curve strength");
    TalonUtil.checkError(elevatorLeaderMotor.configMotionCruiseVelocity(MAX_VELOCITY, Constants.CAN_TIMEOUT),
        "Failed to set elevator motor cruise velocity");
    TalonUtil.checkError(
      elevatorLeaderMotor.configAllowableClosedloopError(0, ALLOWABLE_ERROR, Constants.CAN_TIMEOUT),
        "Failed to set elevator follower motor allowable error");

    TalonUtil.checkError(elevatorLeaderMotor.configClearPositionOnLimitR(true, Constants.CAN_TIMEOUT),
        "Failed to set elevator leader motor clear position on limit R");
    TalonUtil.checkError(
        elevatorLeaderMotor.configVelocityMeasurementWindow(SENSOR_VELOCITY_MEAS_WINDOW,
            Constants.CAN_TIMEOUT),
        "Failed to set elevator leader motor velocity measurement window");


    elevatorLeaderMotor.enableVoltageCompensation(true);
    elevatorLeaderMotor.overrideLimitSwitchesEnable(true);
    elevatorLeaderMotor.overrideSoftLimitsEnable(true);
    elevatorLeaderMotor.setNeutralMode(NEUTRAL_MODE);
    elevatorLeaderMotor.selectProfileSlot(0, 0);
    elevatorLeaderMotor.setInverted(MOTOR_INVERTED);

  }

  private void initializeLog() {
  }

}
