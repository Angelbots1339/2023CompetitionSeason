// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class CancelableSwerveController extends PPSwerveControllerCommand {

  private final Timer timer = new Timer();
  private double timeToCancel;

  
  /**
   * Constructs a new PPSwerveControllerCommand that when executed will follow the provided
   * trajectory. This command will not return output voltages but rather raw module states from the
   * position controllers which need to be put into a velocity PID.
   *
   * <p>Note: The controllers will *not* set the output to zero upon completion of the path- this is
   * left to the user, since it is not appropriate for paths with nonstationary endstates.
   *
   * @param timeToCancel The time after which the path will be canceled, leave zero to never be canceled
   * @param trajectory The trajectory to follow.
   * @param poseSupplier A function that supplies the robot pose - use one of the odometry classes
   *     to provide this.
   * @param kinematics The kinematics for the robot drivetrain.
   * @param xController The Trajectory Tracker PID controller for the robot's x position.
   * @param yController The Trajectory Tracker PID controller for the robot's y position.
   * @param rotationController The Trajectory Tracker PID controller for angle for the robot.
   * @param outputModuleStates The raw output module states from the position controllers.
   * @param requirements The subsystems to require.
   */
  public CancelableSwerveController(
    double timeToCancel,
    PathPlannerTrajectory trajectory,
    Supplier<Pose2d> poseSupplier,
    SwerveDriveKinematics kinematics,
    PIDController xController,
    PIDController yController,
    PIDController rotationController,
    Consumer<SwerveModuleState[]> outputModuleStates,
    Subsystem... requirements
  ) {
    // Use addRequirements() here to declare subsystem dependencies.
    super(trajectory, poseSupplier, kinematics, xController, yController, rotationController, outputModuleStates, requirements);

    this.timer.reset();
    this.timer.start();

    this.timeToCancel = timeToCancel;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return this.timer.hasElapsed(this.timeToCancel);
  }
}
