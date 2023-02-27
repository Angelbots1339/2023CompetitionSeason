// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.objectManipulation.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.FieldDependentConstants;
import frc.robot.subsystems.Intake;

public class RunIntakeForCone extends CommandBase {

  private final Intake intakeAndShooter;
  private final double percent;
  /** Creates a new runIntakeForFallenCone. */
  public RunIntakeForCone(Intake intake, double percent) {
    this.intakeAndShooter = intake;
    this.percent = percent;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.

  private Timer centerTimer = new Timer();
  @Override
  public void execute() {
    intakeAndShooter.runIntakeAtPercent(percent);
    if(intakeAndShooter.coneInIntake())
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
    return centerTimer.get() >= FieldDependentConstants.CurrentField.CONE_SETTLE_TIME;
  }
}
