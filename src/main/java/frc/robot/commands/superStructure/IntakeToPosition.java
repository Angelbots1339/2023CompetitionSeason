// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superStructure;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.ElevatorWristState;

import static frc.robot.Constants.ElevatorWristStateConstants.*;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorWristStateConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Elevator;
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

    if (elevator.getHeightMeters() <= IGNORE_CRUSH_ANGLE_HEIGHT
        && wrist.getAngleDeg() >= CRUSH_ANGLE.getDegrees()
        && heightGoal <= IGNORE_CRUSH_ANGLE_HEIGHT)
      runElevator = false;

    if (wristGoal.getDegrees() <= SAFE_LIMELIGHT_ANGLE.getDegrees()
        && willPassThroughLimeLight(heightGoal, elevator.getHeightMeters()))
      wristGoal = SAFE_LIMELIGHT_ANGLE;

    if (runElevator)
      elevator.setMotionMagicClicks(ElevatorConstants.metersToClicks(heightGoal));

    wrist.setMotionMagic(WristConstants.radiansToClicks(wristGoal.getRadians()));

  }

  private boolean willPassThroughLimeLight(double goal, double current) {
    return (LIMELIGHT_CRUSH_HEIGHT_LOW >= goal && LIMELIGHT_CRUSH_HEIGHT_HIGH <= current)
        || (LIMELIGHT_CRUSH_HEIGHT_HIGH <= goal && LIMELIGHT_CRUSH_HEIGHT_LOW >= current);
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

  public static IntakeToPosition home(Wrist wrist, Elevator elevator) {
    return new IntakeToPosition(wrist, elevator, ElevatorWristStateConstants.HOME);
  }

  public static IntakeToPosition reachHeightBeforeStartAngle(Wrist wrist, Elevator elevator,
      ElevatorWristState finalPos, double heightBeforeStartAngle) {
    return new IntakeToPosition(wrist, elevator, () -> {
      if (elevator.getHeightMeters() < heightBeforeStartAngle)
        return new ElevatorWristState(ElevatorWristStateConstants.HOME.angle.getDegrees(), finalPos.height);
      else
        return finalPos;
    });
  }


  static ShuffleboardTab poseFinder = Shuffleboard.getTab("poseFinder");

  public static GenericEntry poseFinderElevator =  poseFinder.add("height: " , 0).withWidget(BuiltInWidgets.kNumberSlider)
   .withProperties(Map.of("min", 0, "max", 1.33, "Block increment", 0.01)).getEntry();


   public static GenericEntry poseFinderWrist = poseFinder.add("angle: ", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 13, "max", 204.462891, "Block increment", 1)).getEntry();


      public static GenericEntry poseFinderHeightBeforeStartAngle = poseFinder.add("heightBeforeStartAngle: " , 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1.33, "Block increment", 0.05)).getEntry();
   


  public static IntakeToPosition poseFinder(Wrist wrist, Elevator elevator, ElevatorWristState finalPos,
      double heightBeforeStartAngle, String name) {

      

   

    

    return new IntakeToPosition(wrist, elevator, () -> {
      if (elevator.getHeightMeters() < poseFinderHeightBeforeStartAngle.getDouble(heightBeforeStartAngle))
        return new ElevatorWristState(ElevatorWristStateConstants.HOME.angle.getDegrees(),
            poseFinderElevator.getDouble(finalPos.height));
      else
        return new ElevatorWristState(poseFinderWrist.getDouble(finalPos.angle.getDegrees()),
            poseFinderElevator.getDouble(finalPos.height));
    });
  }

  public static IntakeToPosition poseFinder(Wrist wrist, Elevator elevator, ElevatorWristState finalPose, String name) {
    return poseFinder(wrist, elevator, finalPose, 0, name);
  }
}
