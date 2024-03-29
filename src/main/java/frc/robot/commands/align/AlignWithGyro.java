// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.align;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.util.FieldUtil;
import frc.robot.FieldDependentConstants;
import frc.robot.subsystems.Swerve;

public class AlignWithGyro extends CommandBase {
  /** Creates a new AlignWithGyro. */


  Swerve swerve;
  boolean backOn; 

  Timer minTimer = new Timer();
  public AlignWithGyro(Swerve swerve, Boolean backOn) {
    this.swerve =swerve;
    this.backOn = backOn;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    minTimer.start();
    
  
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xOut = 0;
    if(Math.abs(swerve.getPitch()) > FieldDependentConstants.CurrentField.CHARGE_STATION_MAX_ANGLE){
      xOut = (backOn? -1 : 1) * Math.signum(swerve.getGyro().getY()) * 0.32;
    }
    else if(minTimer.get() < 1){
      xOut = (backOn? -1 : 1) * Math.signum(swerve.getGyro().getY()) * 0.32;
    }
    else{
     //xOut = 0;
    }


    swerve.drive(new Translation2d(
               xOut, 0),0, true, false);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(swerve.getPitch()) < FieldDependentConstants.CurrentField.CHARGE_STATION_MIN_ANGLE && (minTimer.get() > 1);
  }
}
