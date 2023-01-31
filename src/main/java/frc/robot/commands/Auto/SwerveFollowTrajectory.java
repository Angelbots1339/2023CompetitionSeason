// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Auto;


import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Swerve;

public class SwerveFollowTrajectory extends PPSwerveControllerCommand {

  public SwerveFollowTrajectory(PathPlannerTrajectory traj, boolean isFirstPath, Swerve swerve) {
    super(
        traj,
        swerve::getPose,
        SwerveConstants.KINEMATICS,
        new PIDController(3.2023, 0, 0),
        new PIDController(3.2023, 0, 0),
        new PIDController(AutoConstants.THETA_KP, 0, 0),
        swerve::setModuleStates,
        swerve);

    if (isFirstPath) {
      swerve.resetOdometry(traj.getInitialHolonomicPose());
    }

  }

  public SwerveFollowTrajectory(PathPlannerTrajectory traj, Swerve swerve) {
    this(traj, false, swerve);
  }

  public static PPSwerveControllerCommand SwerveGenerateAndFollowTrajectoryToPoint(PathPoint endPoint, Swerve swerve) {
    PathPlannerTrajectory traj = PathPlanner.generatePath(new PathConstraints(2, 2),
        new PathPoint(swerve.getPose().getTranslation(), swerve.getHeading(), swerve.getYaw(),
            swerve.getCurrentTotalVelocity()),
        endPoint);
    return new SwerveFollowTrajectory(traj, swerve);
  }
  public static Trajectory SwerveGenerateTrajectoryToPoint(PathPoint endPoint, Swerve swerve){
      return PathPlanner.generatePath(new PathConstraints(2, 2),
      new PathPoint(swerve.getPose().getTranslation(), swerve.getHeading(), swerve.getYaw(),
          swerve.getCurrentTotalVelocity()),
      endPoint);
  }

}
