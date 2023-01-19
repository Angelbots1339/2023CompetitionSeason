// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import static frc.robot.Constants.Vision.*;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonTrackedTarget;

import frc.robot.subsystems.Swerve;

public class SimpleAlignToTarget extends CommandBase {
  /** Creates a new SimpleAlignToTarget. */
  Swerve swerve; 
  PhotonCamera photonCamera;       
  public SimpleAlignToTarget(Swerve swerve) {
    photonCamera = new PhotonCamera("Cam1");
    this.swerve = swerve;
    addRequirements(swerve);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  private double testKp = 0;

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    SmartDashboard.putBoolean("target", photonCamera.getLatestResult().hasTargets());
    SmartDashboard.putBoolean("connected", photonCamera.isConnected());



    if(photonCamera.getLatestResult().hasTargets()){
      PhotonTrackedTarget target = photonCamera.getLatestResult().getBestTarget();
      SmartDashboard.putNumber("yaw ", target.getYaw());
      //swerve.drive(new Translation2d(), target.getYaw() * testKp , false, false);
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
