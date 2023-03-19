// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.align;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.lib.math.ClosedLoopUtil;
import frc.lib.util.FieldUtil;
import frc.robot.FieldDependentConstants;
import frc.robot.Constants.FieldConstants;
import static frc.robot.Constants.SwerveConstants.DrivePidConstants.*;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignOnChargingStation extends PIDCommand {

  public Swerve swerve;


  /** Creates a new AlignOnChargingStation. */
  public AlignOnChargingStation(Swerve swerve) {
    super(
        // The controller that the command will use
        new PIDController(1.5, 0, 0),
        // This should return the measurement
        () -> swerve.getPose().getX(),
        // This should return the setpoint (can also be a constant)
       FieldConstants.RED_ORIGIN.getX()/2, // Offset
        // from
        // Target
        // This uses the output

        output -> {

          swerve.angularDrive(new Translation2d((
               FieldUtil.isBlueAlliance()? 1 : -1 *(MathUtil.clamp(output + ClosedLoopUtil.positionFeedForward(output, 0.3), -1, 1))), 0),
              FieldUtil.getTowardsDriverStation(), true, false);
          // Use the output here
        });
        this.swerve = swerve;

    addRequirements(swerve);
    getController().setTolerance(TRANSLATION_PID_TOLERANCE);
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  // Returns true when the command should end
  @Override
  public boolean isFinished() {
    return Math.abs(swerve.getPitch()) > 6;
    
  }
}
