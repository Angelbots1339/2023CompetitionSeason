// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.objectManipulation.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeAndShooter;

public class RunIntakeForFallenCone extends CommandBase {

  IntakeAndShooter intakeAndShooter;
  /** Creates a new runIntakeForFallenCone. */
  public RunIntakeForFallenCone(IntakeAndShooter intakeAndShooter) {
    this.intakeAndShooter = intakeAndShooter;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.

  private Timer centerTimer = new Timer();
  @Override
  public void execute() {
    intakeAndShooter.runIntakeAtPercent(0, IntakeConstants.INTAKE_CONE_PERCENT);
    if(intakeAndShooter.fallenConeInRange())
      centerTimer.start();
      else{
      centerTimer.stop();
      centerTimer.reset();
      }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeAndShooter.disable();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() { 
    return centerTimer.get() >= IntakeConstants.CENTER_TIME;
  }
}
