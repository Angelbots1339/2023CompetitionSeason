// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superStructre;

import java.sql.Time;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.WristConstants;
import frc.robot.subsystems.Wrist;

public class wristToRotation extends CommandBase {

  private final Wrist wrist;
  private final Supplier<Rotation2d> angle;
  /** Creates a new wristToRotation. */
  public wristToRotation(Wrist wrist, Supplier<Rotation2d> angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.wrist = wrist;
    this.angle = angle;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  private boolean inTolerance;
  private Timer timeInTolerance = new Timer();
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.setMotionMagic(WristConstants.radiansToClicks(angle.get().getRadians()));
    if(Math.abs(wrist.getAngle().minus(angle.get()).getDegrees()) <= WristConstants.MOTION_MAGIC_ERROR_THRESHOLD){
      if(!inTolerance){
        inTolerance = true;
        timeInTolerance.start();
      }
    }
    else if(inTolerance) {
      inTolerance = false;
      timeInTolerance.stop();
      timeInTolerance.reset();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timeInTolerance.get() >= WristConstants.TIME_TO_SETTLE;
  }
}
