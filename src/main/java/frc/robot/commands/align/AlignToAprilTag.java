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
    Pose2d closetNode = swerve.getClosestCubeNode();
    finalDesiredPose = closetNode.transformBy(new Transform2d(
        new Translation2d(FieldDependentConstants.CurrentField.CUBE_ALIGN_OFFSET, 0), Rotation2d.fromDegrees(180)));

  }

  boolean hadTarget = false;
  int lastTarget = 0;

  // Called every time the scheduler runs while the command is scheduled.
 
  Pose2d finalDesiredPose = new Pose2d();

  PIDController xController = new PIDController(1.5, 0, 0);
  PIDController yController = new PIDController(1.5, 0, 0);

  boolean blueSide = false;
  boolean passedFirstAlign = false;

  @Override
  public void execute() {

    /// int id = target.getFiducialId();

    passedFirstAlign = true;

    double alignOffset = FieldDependentConstants.CurrentField.CUBE_ALIGN_OFFSET;

    Pose2d closetNode = swerve.getClosestCubeNode();
    blueSide = closetNode.getX() < 8.26;
    finalDesiredPose = closetNode.transformBy(new Transform2d(
        new Translation2d(FieldDependentConstants.CurrentField.CUBE_ALIGN_OFFSET, 0), Rotation2d.fromDegrees(180)));

    Pose2d firstDesiredPose = closetNode.transformBy(new Transform2d(
        new Translation2d(FieldDependentConstants.CurrentField.CUBE_FIRST_ALIGN_OFFSET, 0),
        Rotation2d.fromDegrees(180)));

    double Y = (blueSide ? 1 : -1) * (yController.calculate(swerve.getPose().getY(), finalDesiredPose.getY())
        + ClosedLoopUtil.positionFeedForward(yController.getPositionError(), DrivePidConstants.TRANSLATION_KS));
    Y = ClosedLoopUtil.stopAtSetPoint(Y, yController.getPositionError(),
        DrivePidConstants.TRANSLATION_PID_TOLERANCE);
    Y = ClosedLoopUtil.clampMaxEffort(Y, VisionConstants.LIMELIGHT_ALIGN_MAX_SPEED);

    double X = 0;
    if (Y > 0.02) {
      X = (blueSide ? 1 : -1) * (xController.calculate(swerve.getPose().getX(), firstDesiredPose.getX())
          + ClosedLoopUtil.positionFeedForward(xController.getPositionError(), DrivePidConstants.TRANSLATION_KS));
      X = ClosedLoopUtil.stopAtSetPoint(X, Math.abs(alignOffset),
          DrivePidConstants.TRANSLATION_PID_TOLERANCE);
    } else if (blueSide ? swerve.getPose().getX() > finalDesiredPose.getX()
        : swerve.getPose().getX() < finalDesiredPose.getX()) {
      X = (blueSide ? 1 : -1) * (xController.calculate(swerve.getPose().getX(), finalDesiredPose.getX())
          + ClosedLoopUtil.positionFeedForward(xController.getPositionError(), DrivePidConstants.TRANSLATION_KS));
    }

    X = ClosedLoopUtil.clampMaxEffort(X, VisionConstants.LIMELIGHT_ALIGN_MAX_SPEED);

    // SmartDashboard.putNumber("X", X);
    // SmartDashboard.putNumber("X feed",
    // ClosedLoopUtil.positionFeedForward(xController.getPositionError(),
    // DrivePidConstants.TRANSLATION_KS));
    // SmartDashboard.putNumber("X out",
    // xController.calculate(swerve.getPose().getX(), desiredPose.getX()));


    swerve.angularDrive(new Translation2d(X, Y), finalDesiredPose.getRotation(), true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.disable();
    System.out.println("end comand" + interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return (blueSide? swerve.getPose().getX() < finalDesiredPose.getX() :
    // swerve.getPose().getX() > finalDesiredPose.getX())
    // && Math.abs(swerve.getPose().getY() - desiredPose.getY()) <
    // FieldDependentConstants.CurrentField.CUBE_ALIGN_Y_TOLERANCE;
    return (blueSide ? swerve.getPose().getX() < finalDesiredPose.getX()
        : swerve.getPose().getX() > finalDesiredPose.getX())
        && Math.abs(
            swerve.getPose().getY() - finalDesiredPose.getY()) < FieldDependentConstants.CurrentField.CUBE_ALIGN_Y_TOLERANCE;
  }
}
