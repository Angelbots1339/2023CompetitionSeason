// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.align;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.VisionConstants.*;

import org.photonvision.targeting.PhotonTrackedTarget;

import frc.lib.math.ClosedLoopUtil;
import frc.robot.Constants;
import frc.robot.FieldDependentConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.SwerveConstants.DrivePidConstants;
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
  int lastTarget = 0;

  // Called every time the scheduler runs while the command is scheduled.
  Pose2d desiredPose = new Pose2d();

  PIDController xController = new PIDController(1.5, 0, 0);
  PIDController yController = new PIDController(1.5, 0, 0);

  boolean blueSide = false;

  @Override
  public void execute() {


      /// int id = target.getFiducialId();

      Pose2d closetNode = swerve.getClosestCubeNode();


    

      double alignOffset = FieldDependentConstants.CurrentField.CUBE_ALIGN_OFFSET;
      if (Math.abs(swerve.getPose().getY() - closetNode.getY()) > FieldDependentConstants.CurrentField.CUBE_ALIGN_Y_TOLERANCE) {
        alignOffset = FieldDependentConstants.CurrentField.CUBE_FIRST_ALIGN_OFFSET;
      }

      if (closetNode.getX() < 8.26) {
        blueSide = true;
        desiredPose = closetNode.transformBy(new Transform2d(
            new Translation2d(alignOffset, 0), Rotation2d.fromDegrees(180)));

      } else  {
        desiredPose = closetNode.transformBy(new Transform2d(
            new Translation2d(-alignOffset, 0), Rotation2d.fromDegrees(180)));

      } 

    

      double X = xController.calculate(swerve.getPose().getX(), desiredPose.getX())
          + ClosedLoopUtil.positionFeedForward(xController.getPositionError(), DrivePidConstants.TRANSLATION_KS);
      if (Math.abs(swerve.getPose().getY() - closetNode.getY()) > FieldDependentConstants.CurrentField.CUBE_ALIGN_Y_TOLERANCE) {
        X = ClosedLoopUtil.stopAtSetPoint(X, xController.getPositionError(),
          DrivePidConstants.TRANSLATION_PID_TOLERANCE);
      }
      
      X = ClosedLoopUtil.clampMaxEffort(X, VisionConstants.LIMELIGHT_ALIGN_MAX_SPEED);

      double Y = xController.calculate(swerve.getPose().getY(), desiredPose.getY())
          + ClosedLoopUtil.positionFeedForward(yController.getPositionError(), DrivePidConstants.TRANSLATION_KS);
      Y = ClosedLoopUtil.stopAtSetPoint(Y, yController.getPositionError(),
          DrivePidConstants.TRANSLATION_PID_TOLERANCE);
      Y = ClosedLoopUtil.clampMaxEffort(Y, VisionConstants.LIMELIGHT_ALIGN_MAX_SPEED);


      SmartDashboard.putNumber("X", X);
      SmartDashboard.putNumber("X feed", ClosedLoopUtil.positionFeedForward(xController.getPositionError(), DrivePidConstants.TRANSLATION_KS));
      SmartDashboard.putNumber("X out", xController.calculate(swerve.getPose().getX(), desiredPose.getX()));
      SmartDashboard.putNumber("Y feed", xController.getPositionError() );
      SmartDashboard.putNumber("Y out", Y);
      swerve.angularDrive(new Translation2d(X, Y), desiredPose.getRotation(), true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return blueSide? swerve.getPose().getX() < desiredPose.getX() : swerve.getPose().getX() > desiredPose.getX()
        && Math.abs(swerve.getPose().getY() - desiredPose.getY()) < FieldDependentConstants.CurrentField.CUBE_ALIGN_Y_TOLERANCE
        && Math.abs(swerve.getYaw().getDegrees() - desiredPose.getRotation().getDegrees()) < DrivePidConstants.ANGLE_TOLERANCE;

  }
}
