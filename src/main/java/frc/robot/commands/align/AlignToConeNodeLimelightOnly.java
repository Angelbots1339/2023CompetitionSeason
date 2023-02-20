// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.align;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.ClosedLoopUtil;
import frc.lib.util.FelidUtil;
import frc.lib.util.LimeLight;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.AlignConstants;
import frc.robot.Constants.SwerveConstants.DrivePidConstants;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class AlignToConeNodeLimelightOnly extends CommandBase {
  /** Creates a new SimpleAlignToTarget. */
  Swerve swerve;

  private double strafeKP = 0.05;

  PIDController anglController = new PIDController(DrivePidConstants.ANGLE_KP, 0, 0);

  PIDController yController = new PIDController(DrivePidConstants.TRANSLATION_P, 0, 0);

  public AlignToConeNodeLimelightOnly(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  private double xOffset;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LimeLight.hasTargets()) {

      SmartDashboard.putNumber("targetPoseHorizontalOffset", LimeLight.getHorizontalOffset());
      SmartDashboard.putNumber("targetArea", LimeLight.getTargetArea());

      double horizontalOffset = LimeLight.getHorizontalOffset();

      xOffset = Math.abs(swerve.getPose().getX() - FelidUtil.getConeNodeMidX(DriverStation.getAlliance()));
      double yOffset = Math.tan(swerve.getAdjustedYaw().getRadians() + Math.toRadians(horizontalOffset))
          * xOffset;

      double Y = yController.calculate(yOffset, 0)
          + ClosedLoopUtil.positionFeedForward(yController.getPositionError(), SwerveConstants.DRIVE_KS);
      Y = ClosedLoopUtil.stopAtSetPoint(Y, yController.getPositionError(), DrivePidConstants.TRANSLATION_PID_TOLERANCE);

      double X = swerve.pidToX(FelidUtil.getGridAlignX(DriverStation.getAlliance()));
      double rotation = anglController.calculate(horizontalOffset, 0)
          + ClosedLoopUtil.positionFeedForward(anglController.getPositionError(), DrivePidConstants.ANGLE_KS);

      rotation = ClosedLoopUtil.stopAtSetPoint(rotation, anglController.getPositionError(), DrivePidConstants.ANGLE_TOLERANCE);
      X = Math.abs(xOffset - AlignConstants.CONE_NODE_X_OFFSET) >= DrivePidConstants.TRANSLATION_PID_TOLERANCE ? X : 0;

      swerve.drive(new Translation2d(X, Y), rotation, true, false);
      swerve.drive(new Translation2d(0, Y), rotation, true, false);

    } else {
      swerve.disable();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(xOffset - AlignConstants.CONE_NODE_X_OFFSET) <= DrivePidConstants.TRANSLATION_PID_TOLERANCE
        && ClosedLoopUtil.inSetPoint(anglController.getPositionError(), DrivePidConstants.ANGLE_TOLERANCE)
        && ClosedLoopUtil.inSetPoint(yController.getPositionError(), DrivePidConstants.TRANSLATION_PID_TOLERANCE);
  }

}
