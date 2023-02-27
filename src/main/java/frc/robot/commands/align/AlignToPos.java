// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.SwerveConstants.DrivePidConstants;
import frc.robot.subsystems.Swerve;

public class AlignToPos extends CommandBase {
  private final Swerve swerve;
  private final Pose2d target;

  private double xTolerance, yTolerance;
  /** Creates a new AlignToPos. */
  public AlignToPos(Swerve swerve, Pose2d target, double xTolerance, double yTolerance) {
    this.swerve = swerve;
    this.target = target;
    this.xTolerance = xTolerance;
    this.yTolerance = yTolerance;
    addRequirements(swerve);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    swerve.pidToPose(target);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    Pose2d error = swerve.getPose().relativeTo(target); 
    return Math.abs(error.getX()) < xTolerance && Math.abs(error.getY()) < yTolerance && Math.abs(error.getRotation().getDegrees()) < DrivePidConstants.ANGLE_TOLERANCE;
  }
}
