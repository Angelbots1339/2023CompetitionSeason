// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.VisionConstants.*;

import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.FieldDependentConstants;
import frc.robot.subsystems.Swerve;

public class AlignToAprilTag extends CommandBase {
  /** Creates a new SimpleAlignToTarget. */
  private final Swerve swerve;

  public AlignToAprilTag(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  boolean hadTarget = false;

  // Called every time the scheduler runs while the command is scheduled.
  Pose2d desiredPose;

  @Override
  public void execute() {

    if (APRILTAG_CAM.getLatestResult().hasTargets()) {
      PhotonTrackedTarget target = APRILTAG_CAM.getLatestResult().getBestTarget();
      int id = target.getFiducialId();
      double alignOffset = FieldDependentConstants.CurrentField.CUBE_ALIGN_OFFSET;
      if(Math.abs(swerve.getPose().getY() - desiredPose.getY()) > FieldDependentConstants.CurrentField.CUBE_ALIGN_Y_TOLERANCE){
        alignOffset = FieldDependentConstants.CurrentField.CUBE_ALIGN_X_OFFSET;
      }

      if (id >= 6) {
        desiredPose = swerve.getField().getTagPose(id).get().toPose2d().transformBy(new Transform2d(
            new Translation2d(alignOffset, 0), Rotation2d.fromDegrees(0)));
      } else if (id <= 3) {
        desiredPose = swerve.getField().getTagPose(id).get().toPose2d().transformBy(new Transform2d(
            new Translation2d(-alignOffset, 0), Rotation2d.fromDegrees(0)));
      } else
        return;

      
      swerve.pidToPose(desiredPose);
      hadTarget = true;
    }
    else if(hadTarget){
      swerve.pidToPose(desiredPose);
    }

    swerve.disable();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(swerve.getPose().getX() - desiredPose.getX()) < FieldDependentConstants.CurrentField.CUBE_ALIGN_Y_TOLERANCE
        && Math.abs(swerve.getPose().getY() - desiredPose.getY()) < FieldDependentConstants.CurrentField.CUBE_ALIGN_Y_TOLERANCE;
  }
}
