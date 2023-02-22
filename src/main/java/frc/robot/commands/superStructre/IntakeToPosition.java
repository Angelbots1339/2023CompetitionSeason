// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superStructre;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.Candle;
import frc.lib.util.ElevatorWristState;
import frc.lib.util.Candle.LEDState;

import static frc.robot.Constants.ElevatorWristStateConstants.*;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorWristStateConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class IntakeToPosition extends CommandBase {
  protected Wrist wrist;
  protected Elevator elevator;
  protected Supplier<ElevatorWristState> goal;

  /** Creates a new IntakeToPosition. */
  public IntakeToPosition(Wrist wrist, Elevator elevator, Supplier<ElevatorWristState> state) {
    this.wrist = wrist;
    this.elevator = elevator;
    this.goal = state;
    addRequirements(wrist, elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public IntakeToPosition(Wrist wrist, Elevator elevator, ElevatorWristState state) {
    this.wrist = wrist;
    this.elevator = elevator;
    this.goal = () -> state;
    addRequirements(wrist, elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    boolean runElevator = true;
    Rotation2d wristGoal = goal.get().angle;
    double heightGoal = goal.get().height;

    if (elevator.getHeight() <= IGNORE_CRUSH_ANGLE_HEIGHT
        && wrist.getAngleDeg() >= CRUSH_ANGLE.getDegrees()
        && heightGoal <= IGNORE_CRUSH_ANGLE_HEIGHT) 
      runElevator = false;
    
    if (wristGoal.getDegrees() <= SAFE_LIMELIGHT_ANGLE.getDegrees() && willPassThroughLimeLight(heightGoal, elevator.getHeight())) 
        wristGoal = SAFE_LIMELIGHT_ANGLE;  
  
    if(runElevator)
      elevator.setMotionMagicClicks(ElevatorConstants.metersToClicks(heightGoal));
    wrist.setMotionMagic(WristConstants.radiansToClicks(wristGoal.getRadians()));
  }


  private boolean willPassThroughLimeLight(double goal, double current) {
    return (LIMELIGHT_CRUSH_HEIGHT_LOW >= goal && LIMELIGHT_CRUSH_HEIGHT_HIGH <= current) || (LIMELIGHT_CRUSH_HEIGHT_HIGH <= goal && LIMELIGHT_CRUSH_HEIGHT_LOW >= current);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.disable();
    elevator.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }


  public static Command home(Wrist wrist, Elevator elevator) {
    return new IntakeToPosition(wrist, elevator, ElevatorWristStateConstants.HOME);
  }
}
