// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import java.util.ArrayList;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.lib.util.Auto.AutoUtils;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class examplePathPlannerAuto extends SequentialCommandGroup {
  /** Creates a new examplePathPlannerAuto. */
  public examplePathPlannerAuto(Swerve swerve) {
    // ArrayList<PathPlannerTrajectory> trajectories =
    // AutoUtils.loadTrajectoriesWithConstraints("TestPath");
    PathPlannerTrajectory trajectory = PathPlanner.loadPath("TestPath", PathPlanner.getConstraintsFromPath("TestPath"));

    addRequirements(swerve);
    addCommands(
        swerve.followTrajectoryCommand(trajectory, true));

  }
}
