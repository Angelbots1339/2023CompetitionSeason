// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.align;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
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


  private final DoubleSupplier coneOffset;
  private final boolean favorHigh;


  PIDController yController = new PIDController(0.5, 0, 0);
  PIDController xController = new PIDController(1.5, 0, 0);

  

  public AlignToConeNodeLimelightOnly(Swerve swerve, DoubleSupplier coneOffset, boolean favorHigh) {
    this.coneOffset = coneOffset;
    this.swerve = swerve;
    this.favorHigh = favorHigh;
    addRequirements(swerve);
  }


 


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    RetroReflectiveTargeter.update(swerve.getPose(), favorHigh);
    if (LimeLight.hasTargets()) {
      double yOffset = RetroReflectiveTargeter.getYOffsetFromConeOffset(swerve.getPose(), coneOffset.getAsDouble());
      double xOffset = RetroReflectiveTargeter.getXOffsetFromPlacePos();

      double xSetPoint = 0;

      if(Math.abs(yOffset) > FieldDependentConstants.CurrentField.LIMELIGHT_ALIGN_Y_TOLERANCE){
        if(RetroReflectiveTargeter.getStatus() == targetingStatus.HIGH){
          xSetPoint = FieldDependentConstants.CurrentField.HIGH_NODE_LIMELIGHT_ALIGN_OFFSET;
        } else {
          xSetPoint = FieldDependentConstants.CurrentField.MID_NODE_LIMELIGHT_ALIGN_OFFSET;
        }
      } 
      

    double X = xController.calculate(xOffset, xSetPoint) 
       + ClosedLoopUtil.positionFeedForward(xController.getPositionError(), DrivePidConstants.TRANSLATION_KV);
      X = ClosedLoopUtil.stopAtSetPoint(X, xController.getPositionError(), FieldDependentConstants.CurrentField.LIMELIGHT_ALIGN_X_TOLERANCE);
      X = ClosedLoopUtil.clampMaxEffort(X, VisionConstants.LIMELIGHT_ALIGN_MAX_SPEED);

    double Y = yController.calculate(yOffset, 0) 
       + ClosedLoopUtil.positionFeedForward(yController.getPositionError(), DrivePidConstants.TRANSLATION_KV);
       Y = ClosedLoopUtil.stopAtSetPoint(X, xController.getPositionError(), FieldDependentConstants.CurrentField.LIMELIGHT_ALIGN_Y_TOLERANCE);
       Y = ClosedLoopUtil.clampMaxEffort(Y, VisionConstants.LIMELIGHT_ALIGN_MAX_SPEED);

      swerve.angularDrive(new Translation2d(X,-Y), FieldUtil.getTowardsDriverStation(), true, true);
 
    } else {
      swerve.disable();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LimeLight.setLedMode(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return Math.abs(yController.getPositionError()) < FieldDependentConstants.CurrentField.LIMELIGHT_ALIGN_Y_TOLERANCE
        && Math.abs(xController.getPositionError()) < FieldDependentConstants.CurrentField.LIMELIGHT_ALIGN_X_TOLERANCE
        && LimeLight.hasTargets();
  }

}
