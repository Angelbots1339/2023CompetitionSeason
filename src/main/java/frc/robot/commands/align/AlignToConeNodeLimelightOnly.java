// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.align;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.ClosedLoopUtil;
import frc.lib.util.FieldUtil;
import frc.lib.util.LimeLight;
import frc.robot.FieldDependentConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.SwerveConstants.DrivePidConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.vision.RetroReflectiveTargeter;
import frc.robot.vision.RetroReflectiveTargeter.targetingStatus;

/** Add your docs here. */
public class AlignToConeNodeLimelightOnly extends CommandBase {
  /** Creates a new SimpleAlignToTarget. */
  private final Swerve swerve;

  private Timer minTimer = new Timer();
  private final DoubleSupplier coneOffset;
  private final boolean favorHigh;
  private double xSetPoint = 0;

  PIDController yController = new PIDController(3.5, 0, 0);
  PIDController xController = new PIDController(3.5, 0, 0);

  public AlignToConeNodeLimelightOnly(Swerve swerve, DoubleSupplier coneOffset, boolean favorHigh) {
    this.coneOffset = coneOffset;
    this.swerve = swerve;
    this.favorHigh = favorHigh;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    minTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RetroReflectiveTargeter.update(swerve.getPose(), favorHigh);
    if (LimeLight.hasTargets()) {
      double yOffset = RetroReflectiveTargeter.getYOffset();
      // yOffset = RetroReflectiveTargeter.getYOffset();
      double xOffset = RetroReflectiveTargeter.getXOffset();

      double firstXSetPoint = 0;
      xSetPoint = 0;

      if (RetroReflectiveTargeter.getStatus() == targetingStatus.HIGH) {
        firstXSetPoint = FieldDependentConstants.CurrentField.HIGH_NODE_LIMELIGHT_FIRST_ALIGN_OFFSET;
        xSetPoint = FieldDependentConstants.CurrentField.HIGH_NODE_LIMELIGHT_ALIGN_OFFSET;

      } else {
        firstXSetPoint = FieldDependentConstants.CurrentField.MID_NODE_LIMELIGHT_FIRST_ALIGN_OFFSET;
        xSetPoint = FieldDependentConstants.CurrentField.MID_NODE_LIMELIGHT_ALIGN_OFFSET;

      }

      double Y = yController.calculate(yOffset, 0)
          + ClosedLoopUtil.positionFeedForward(yController.getPositionError(), DrivePidConstants.TRANSLATION_KS);
      Y = ClosedLoopUtil.stopAtSetPoint(Y, yController.getPositionError(),
          FieldDependentConstants.CurrentField.LIMELIGHT_ALIGN_Y_TOLERANCE);
      Y = ClosedLoopUtil.clampMaxEffort(Y, VisionConstants.LIMELIGHT_ALIGN_MAX_SPEED);

      double X = 0;
      if (Y > 0.06) {
        X = xController.calculate(xOffset, firstXSetPoint)
            + ClosedLoopUtil.positionFeedForward(xController.getPositionError(), DrivePidConstants.TRANSLATION_KS);
        X = ClosedLoopUtil.stopAtSetPoint(X, xController.getPositionError(),
            0.04);
      } else {
        X = xController.calculate(xOffset, xSetPoint)
            + ClosedLoopUtil.positionFeedForward(xController.getPositionError(), DrivePidConstants.TRANSLATION_KS);
        if (RetroReflectiveTargeter
            .getXOffset() < xSetPoint) {
          X = 0;
        }
      }

      X = ClosedLoopUtil.clampMaxEffort(X, VisionConstants.LIMELIGHT_ALIGN_MAX_SPEED);

      // SmartDashboard.putNumber("y Error", yController.getPositionError());

      SmartDashboard.putNumber("X offest", xOffset);

      // SmartDashboard.putNumber("y out", yController.calculate(yOffset, 0));
      // SmartDashboard.putNumber("y feed",
      // ClosedLoopUtil.positionFeedForward(yController.getPositionError(),
      // DrivePidConstants.TRANSLATION_KS));

      SmartDashboard.putBoolean("y setPoint",
          yController.getPositionError() < FieldDependentConstants.CurrentField.LIMELIGHT_ALIGN_Y_TOLERANCE);
      SmartDashboard.putBoolean("x setPoint",
          RetroReflectiveTargeter.getXOffset() < (RetroReflectiveTargeter.getStatus() == targetingStatus.HIGH
              ? FieldDependentConstants.CurrentField.HIGH_NODE_LIMELIGHT_ALIGN_OFFSET
              : FieldDependentConstants.CurrentField.MID_NODE_LIMELIGHT_ALIGN_OFFSET));

      swerve.angularDrive(new Translation2d(X, -Y), FieldUtil.getTowardsDriverStation(), true, true);
    } else {
      swerve.disable();

    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return Math
        .abs(RetroReflectiveTargeter.getYOffsetFromConeOffset(swerve.getPose(),
            coneOffset.getAsDouble())) < FieldDependentConstants.CurrentField.LIMELIGHT_ALIGN_Y_TOLERANCE
        && RetroReflectiveTargeter.getXOffset() < xSetPoint
        && LimeLight.hasTargets()
        && minTimer.get() > 0.3;
    // Math.abs(swerve.getAdjustedYaw().getDegrees() - 180) < 4;
  }

}
