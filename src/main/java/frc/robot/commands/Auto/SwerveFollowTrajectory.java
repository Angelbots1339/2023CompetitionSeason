// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;

import static frc.robot.Constants.AutoConstants.*;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

public class SwerveFollowTrajectory extends PPSwerveControllerCommand {

  public SwerveFollowTrajectory(PathPlannerTrajectory traj, boolean isFirstPath, Swerve swerve, Boolean useVision) {
    super(
        traj,
        useVision ? swerve::getPose : swerve::getPoseOdometry,
        new PIDController(X_KP, 0, 0), // 3.203
        new PIDController(Y_KP, 0, 0),
        new PIDController(THETA_KP, 0, 0),
        swerve::setChassisSpeeds,
        true,
        swerve);

    if (isFirstPath) {
      swerve.resetOdometry(traj.getInitialHolonomicPose());
    }
    // super.s
  }

  public SwerveFollowTrajectory(PathPlannerTrajectory traj, Swerve swerve) {
    this(traj, false, swerve, true);
  }

  public static PPSwerveControllerCommand SwerveGenerateAndFollowTrajectoryToPoint(PathPoint endPoint, Swerve swerve) {
    PathPlannerTrajectory traj = PathPlanner.generatePath(new PathConstraints(2, 2),
        new PathPoint(swerve.getPoseOdometry().getTranslation(), swerve.getHeading(), swerve.getYaw(),
            swerve.getCurrentTotalVelocity()),
        endPoint);
    return new SwerveFollowTrajectory(traj, swerve);
  }

  public static Command FollowTrajectoryWithEvents(PathPlannerTrajectory traj, boolean isFirstPath, Boolean useVision, Swerve swerve,
      Intake intake, Elevator elevator, Wrist wrist) {
    return new FollowPathWithEvents(new SwerveFollowTrajectory(traj, isFirstPath, swerve, useVision), traj.getMarkers(),
        AutoEventMarkers.getMap(intake, elevator, wrist));
  }

  public static Trajectory SwerveGenerateTrajectoryToPoint(PathPoint endPoint, Swerve swerve) {
    return PathPlanner.generatePath(new PathConstraints(2, 2),
        new PathPoint(swerve.getPoseOdometry().getTranslation(), swerve.getHeading(), swerve.getYaw(),
            swerve.getCurrentTotalVelocity()),
        endPoint);
  }

}
