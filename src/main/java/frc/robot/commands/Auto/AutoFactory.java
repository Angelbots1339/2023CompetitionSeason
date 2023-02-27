// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.lib.util.FieldUtil;
import frc.robot.commands.objectManipulation.score.ScoreCommandFactory;
import frc.robot.commands.objectManipulation.score.ScoreCommandFactory.ScoreHight;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

/** Add your docs here. */
public class AutoFactory {

    public static Command Score2(Wrist wrist, Elevator elevator, Intake intake, Swerve swerve) {
        List<PathPlannerTrajectory> trajectories = PathPlanner.loadPathGroup("2ConeGrabBalance", new PathConstraints(3, 1));
        swerve.zeroGyro(FieldUtil.getTwoardsDriverStation().getDegrees());
        return Commands.sequence(
                ScoreCommandFactory.scoreConeNode(wrist, elevator, intake, () -> ScoreHight.HIGH).asProxy(),
                SwerveFollowTrajectory.FollowTrajectoryWithEvents(trajectories.get(0), true, true, swerve, intake,
                        elevator, wrist),
                ScoreCommandFactory.scoreCubeNode(wrist, elevator, intake, () -> ScoreHight.HIGH).asProxy());
    }

}
