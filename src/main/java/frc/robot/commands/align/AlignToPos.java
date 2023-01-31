// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;

public class AlignToPos extends CommandBase {
  private Swerve swerve;
  private boolean atSetpoints = false;
  private Pose2d target;
  /** Creates a new AlignToPos. */
  public AlignToPos(Swerve swerve, Pose2d target) {
    this.swerve = swerve;
    this.target = target;
    addRequirements(swerve);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    atSetpoints = swerve.PIDToPose(target);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return atSetpoints;
  }
}
