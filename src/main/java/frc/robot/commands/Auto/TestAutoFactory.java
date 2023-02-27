// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.auto.AutoUtils;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestAutoFactory {
  /** Creates a new examplePathPlannerAuto. */
  public static Command vision2mTest(Swerve swerve) {
    List<PathPlannerTrajectory> trajectories = AutoUtils.loadTrajectoriesWithConstraints("2mTest");
    return new SwerveFollowTrajectory(trajectories.get(0), true, swerve, true);
  }

  public static Command nonVision2mTest(Swerve swerve) {
    List<PathPlannerTrajectory> trajectories = AutoUtils.loadTrajectoriesWithConstraints("2mTest");
    return new SwerveFollowTrajectory(trajectories.get(0), true, swerve, false);
  }
}
