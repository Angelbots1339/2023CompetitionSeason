// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.align;

import java.lang.annotation.Target;

import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.lib.math.ClosedLoopUtil;
import frc.robot.Constants.FieldConstants;
import static frc.robot.Constants.SwerveConstants.DrivePidConstants.*;

import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignOnChargingStation extends PIDCommand {

  private Swerve swerve;

  /** Creates a new AlignOnChargingStation. */
  public AlignOnChargingStation(Swerve swerve) {
    super(
        // The controller that the command will use
        new PIDController(TRANSLATION_P, 0, 0),
        // This should return the measurement
        () -> swerve.getPose().getX(),
        // This should return the setpoint (can also be a constant)
        DriverStation.getAlliance() == Alliance.Red ? FieldConstants.RED_ORIGIN.getX() - 1.524508 : 1.524508, // Offset
                                                                                                              // from
                                                                                                              // Target
        // This uses the output
        output -> {
          swerve.drive(new Translation2d(
              MathUtil.clamp(output + ClosedLoopUtil.positionFeedForward(output, SwerveConstants.DRIVE_KS), -1, 1), 0),
              0, true, false);
          // Use the output here
        });

    addRequirements(swerve);
    this.swerve = swerve;
    getController().setTolerance(TRANSLATION_PID_TOLERANCE);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(getController().getPositionError()) <= getController().getPositionTolerance();
  }
}
