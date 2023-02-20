// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.align;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.math.ClosedLoopUtil;
import frc.lib.util.FelidUtil;
import frc.lib.util.LimeLight;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.SwerveConstants.DrivePidConstants;
import frc.robot.subsystems.Swerve;

public class ApproachConeNode extends CommandBase {

  Swerve swerve;
  PIDController yController = new PIDController(DrivePidConstants.TRANSLATION_P, 0, 0);

  /** Creates a new AproachConeNode. */
  public ApproachConeNode(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (LimeLight.hasTargets()) {
      double horizontalOffset = LimeLight.getHorizontalOffset();

      double xOffset = Math.abs(swerve.getPose().getX() - FelidUtil.getConeNodeMidX(DriverStation.getAlliance()));
      double yOffset = Math.tan(swerve.getAdjustedYaw().getRadians() + Math.toRadians(horizontalOffset))
          * xOffset;

      double y = yController.calculate(yOffset, 0)
          + ClosedLoopUtil.positionFeedForward(yController.getPositionError(), SwerveConstants.DRIVE_KS);

      double x = swerve.pidToX(FelidUtil.getGridAlignX(DriverStation.getAlliance()));
      y = ClosedLoopUtil.stopAtSetPoint(y, yController.getPositionError(), DrivePidConstants.TRANSLATION_PID_TOLERANCE);

      Rotation2d angle = DriverStation.getAlliance() == Alliance.Red ? Rotation2d.fromDegrees(0)
          : Rotation2d.fromDegrees(180);

      swerve.angularDrive(new Translation2d(ClosedLoopUtil.clampMaxEffort(x, SwerveConstants.MAX_SPEED),
          ClosedLoopUtil.clampMaxEffort(y, SwerveConstants.MAX_SPEED)), angle, true, false);

    } else
      swerve.disable();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerve.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ClosedLoopUtil.inSetPoint(yController.getPositionError(), DrivePidConstants.TRANSLATION_PID_TOLERANCE)
        && swerve.pidToXAtSetPoint(FelidUtil.getGridAlignX(DriverStation.getAlliance()));
  }
}
