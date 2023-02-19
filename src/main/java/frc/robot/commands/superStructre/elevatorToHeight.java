// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superStructre;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

public class elevatorToHeight extends CommandBase {

  private final Elevator elevator;
  private final DoubleSupplier height;
  
  /** Creates a new elevatorToHeight. */
  public elevatorToHeight(Elevator elevator, DoubleSupplier height) {
    this.elevator = elevator;
    this.height = height;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {


    if(Math.abs(elevator.getPositionMeters() - getClampedHeight()) <= ElevatorConstants.MOTION_MAGIC_ERROR_THRESHOLD){
      end(false);
    }
  }


  private Timer timeInTolerance = new Timer();
  private boolean inThreshold = false;
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setMotionMagicClicks(ElevatorConstants.metersToClicks(getClampedHeight()));
    SmartDashboard.putNumber("Elevator Motion Magic", ElevatorConstants.metersToClicks(getClampedHeight()));
    /* Check if closed loop error is within the threshld */
    if(Math.abs(elevator.getPositionMeters() - getClampedHeight()) <= ElevatorConstants.MOTION_MAGIC_ERROR_THRESHOLD){
      if(!inThreshold){
        inThreshold = true;
        timeInTolerance.start();
      }
    }
    else if(inThreshold){
      timeInTolerance.stop();
      timeInTolerance.reset();
      inThreshold = false;
    }

    SmartDashboard.putNumber("true Error", elevator.getPositionMeters() - getClampedHeight());
    SmartDashboard.putBoolean("in threshold", inThreshold);
    
  }

  private double getClampedHeight(){
    return MathUtil.clamp(height.getAsDouble(), ElevatorConstants.clicksToMeters(ElevatorConstants.REVERSE_SOFT_LIMIT), ElevatorConstants.clicksToMeters(ElevatorConstants.FORWARD_SOFT_LIMIT));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timeInTolerance.get() >= ElevatorConstants.TIME_TO_SETTLE;
  }
}
