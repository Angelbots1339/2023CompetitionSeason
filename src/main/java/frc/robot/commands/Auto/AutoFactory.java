// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.lang.reflect.Field;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import frc.robot.FieldDependentConstants;
import frc.robot.commands.align.AlignOnChargingStation;
import frc.robot.commands.align.AlignWithGyro;
import frc.robot.commands.objectManipulation.score.ScoreCommandFactory;
import frc.robot.commands.objectManipulation.score.ScoreCommandFactory.ScoreHight;
import frc.robot.commands.superStructure.IntakePositionCommandFactory;
import frc.robot.commands.superStructure.IntakeToPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import frc.robot.vision.PoseEstimator.PoseEstimationState;

/** Add your docs here. */
public class AutoFactory {

        public static Command Score2BalancePos6(Wrist wrist, Elevator elevator, Intake intake, Swerve swerve) {
                List<PathPlannerTrajectory> trajectories = PathPlanner.loadPathGroup("2Balance6",
                                new PathConstraints(3.1, 1.1), new PathConstraints(3.5, 1.5),
                                new PathConstraints(3.1, 1.1), new PathConstraints(3.1, 1.1));
                return Commands.sequence(
                                resetGyroAndPos(swerve, trajectories.get(0)),
                                ScoreCommandFactory.scoreConeNode(wrist, elevator, intake, () -> ScoreHight.HIGH),
                                SwerveFollowTrajectory.FollowTrajectoryWithEvents(trajectories.get(0), false,
                                                swerve, intake,
                                                elevator, wrist),
                                SwerveFollowTrajectory.FollowTrajectoryWithEvents(trajectories.get(1), false,
                                                swerve, intake,
                                                elevator, wrist),
                                ScoreCommandFactory.alignAndScoreCubeHighNotAsProxy(wrist, elevator, intake,
                                                swerve),
                                SwerveFollowTrajectory.FollowTrajectoryWithEvents(trajectories.get(3), true,
                                                swerve, intake,
                                                elevator, wrist),
                                Commands.parallel(
                                                new AlignWithGyro(swerve, false)
                                                                .andThen(new RunCommand(swerve::xPos, swerve)),
                                                IntakeToPosition.home(wrist, elevator)));
        }

        public static Command Score2BalancePos6NoTurn(Wrist wrist, Elevator elevator, Intake intake, Swerve swerve) {
                List<PathPlannerTrajectory> trajectories = PathPlanner.loadPathGroup("2Balance6NoTurn",
                                new PathConstraints(4, 2.3),
                                new PathConstraints(3.1, 1.1),
                                new PathConstraints(1, 0.5));
                return Commands.sequence(
                                resetGyroAndPos(swerve, trajectories.get(0)),
                                ScoreCommandFactory.scoreConeNode(wrist, elevator, intake, () -> ScoreHight.HIGH),
                                SwerveFollowTrajectory.FollowTrajectoryWithEvents(trajectories.get(0), false,
                                                swerve, intake,
                                                elevator, wrist),
                                ScoreCommandFactory.scoreCubeNode(wrist, elevator, intake, () -> ScoreHight.HIGH),
                                SwerveFollowTrajectory.FollowTrajectoryWithEvents(trajectories.get(1), true,
                                                swerve, intake,
                                                elevator, wrist),
                                SwerveFollowTrajectory.FollowTrajectoryWithEvents(trajectories.get(2), true,
                                                swerve, intake,
                                                elevator, wrist),
                                Commands.parallel(
                                                new AlignWithGyro(swerve, false)
                                                                .andThen(new RunCommand(swerve::xPos, swerve)),
                                                IntakeToPosition.home(wrist, elevator)));
        }

        public static Command Score3Pos6Cube(Wrist wrist, Elevator elevator, Intake intake, Swerve swerve) {
                List<PathPlannerTrajectory> trajectories1 = PathPlanner.loadPathGroup("2Balance6NoTurn",
                                new PathConstraints(4, 2.3));
                List<PathPlannerTrajectory> trajectories2 = PathPlanner.loadPathGroup("3rdPlaceCube6",
                                new PathConstraints(4.4, 3.2));
                return Commands.sequence(
                                resetGyroAndPos(swerve, trajectories1.get(0)),
                                changeVisionState(swerve, PoseEstimationState.NON),
                                ScoreCommandFactory.scoreConeNode(wrist, elevator, intake, () -> ScoreHight.HIGH),
                                SwerveFollowTrajectory.FollowTrajectoryWithEvents(trajectories1.get(0), true,
                                                swerve, intake,
                                                elevator, wrist),
                                ScoreCommandFactory.scoreCubeNode(wrist, elevator, intake, () -> ScoreHight.HIGH),
                                SwerveFollowTrajectory.FollowTrajectoryWithEvents(trajectories2.get(0), true,
                                                swerve, intake,
                                                elevator, wrist),
                                ScoreCommandFactory.scoreCubeNode(wrist, elevator, intake, () -> ScoreHight.MID),
                                IntakeToPosition.home(wrist, elevator));
                           
        }
        public static Command Score3Pos1(Wrist wrist, Elevator elevator, Intake intake, Swerve swerve) {
                List<PathPlannerTrajectory> trajectories1 = PathPlanner.loadPathGroup("2Balance1",
                new PathConstraints(4, 2.4));
                List<PathPlannerTrajectory> trajectories2 = PathPlanner.loadPathGroup("3rdPlaceCube1",
                new PathConstraints(4.4, 3.2));
                return Commands.sequence(
                                resetGyroAndPos(swerve, trajectories1.get(0)),
                                changeVisionState(swerve, PoseEstimationState.CABLE),
                                ScoreCommandFactory.scoreConeNode(wrist, elevator, intake, () -> ScoreHight.HIGH),
                                SwerveFollowTrajectory.FollowTrajectoryWithEvents(trajectories1.get(0), true,
                                                swerve, intake,
                                                elevator, wrist),
                                ScoreCommandFactory.scoreCubeNode(wrist, elevator, intake, () -> ScoreHight.HIGH),
                                SwerveFollowTrajectory.FollowTrajectoryWithEvents(trajectories2.get(0), true,
                                                swerve, intake,
                                                elevator, wrist),
                                ScoreCommandFactory.scoreCubeNode(wrist, elevator, intake, () -> ScoreHight.MID));
        }
        public static Command Score2Grab1(Wrist wrist, Elevator elevator, Intake intake, Swerve swerve) {
                List<PathPlannerTrajectory> trajectories1 = PathPlanner.loadPathGroup("2Balance1",
                new PathConstraints(4, 2));
                List<PathPlannerTrajectory> trajectories2 = PathPlanner.loadPathGroup("3rdGrabCube1",
                new PathConstraints(4, 2));
                return Commands.sequence(
                                resetGyroAndPos(swerve, trajectories1.get(0)),
                                changeVisionState(swerve, PoseEstimationState.CABLE),
                                ScoreCommandFactory.scoreConeNode(wrist, elevator, intake, () -> ScoreHight.HIGH),
                                SwerveFollowTrajectory.FollowTrajectoryWithEvents(trajectories1.get(0), true,
                                                swerve, intake,
                                                elevator, wrist),
                                ScoreCommandFactory.scoreCubeNode(wrist, elevator, intake, () -> ScoreHight.HIGH),
                                SwerveFollowTrajectory.FollowTrajectoryWithEvents(trajectories2.get(0), true,
                                                swerve, intake,
                                                elevator, wrist));
        }
        public static Command test(Wrist wrist, Elevator elevator, Intake intake, Swerve swerve) {
                List<PathPlannerTrajectory> trajectories1 = PathPlanner.loadPathGroup("2Balance1",
                new PathConstraints(4, 2.4));
                List<PathPlannerTrajectory> trajectories2 = PathPlanner.loadPathGroup("3rdPlaceCube1",
                new PathConstraints(4.4, 3.2));
                return Commands.sequence(
                                resetGyroAndPos(swerve, trajectories1.get(0)),
                                changeVisionState(swerve, PoseEstimationState.NON));
        }

        public static Command Shoot5Cubes(Wrist wrist, Elevator elevator, Intake intake, Swerve swerve) {
                List<PathPlannerTrajectory> trajectories = PathPlanner.loadPathGroup("5PlaceBumpSideStart",
                                new PathConstraints(3.1, 1.1));

                List<PathPlannerTrajectory> trajectories2 = PathPlanner.loadPathGroup("5PlaceBumpSideEnd",
                                new PathConstraints(3.1, 1.1));
                return Commands.sequence(
                                resetGyroAndPos(swerve, trajectories.get(0)),
                                ScoreCommandFactory.throwCube(wrist, elevator, intake)
                                                .deadlineWith(new RunCommand(swerve::disable)),
                                SwerveFollowTrajectory.FollowTrajectoryWithEvents(trajectories.get(0), false,
                                                swerve, intake,
                                                elevator, wrist),
                                SwerveFollowTrajectory.FollowTrajectoryWithEvents(trajectories2.get(0), false,
                                                swerve, intake,
                                                elevator, wrist),
                                                IntakeToPosition.home(wrist, elevator));
        }
        



        public static Command ScoreGrabBallance(Wrist wrist, Elevator elevator, Intake intake, Swerve swerve) {
                List<PathPlannerTrajectory> trajectories = PathPlanner.loadPathGroup("2Balance6",
                                new PathConstraints(3.1, 1.1));

                List<PathPlannerTrajectory> trajectories2 = PathPlanner.loadPathGroup("BalanceFromBack",
                                new PathConstraints(3.1, 1.1));
                return Commands.sequence(
                                resetGyroAndPos(swerve, trajectories.get(0)),
                                ScoreCommandFactory.scoreConeNode(wrist, elevator, intake, () -> ScoreHight.HIGH)
                                                .deadlineWith(new RunCommand(swerve::disable)),
                                SwerveFollowTrajectory.FollowTrajectoryWithEvents(trajectories.get(0), false,
                                                swerve, intake,
                                                elevator, wrist),
                                SwerveFollowTrajectory.FollowTrajectoryWithEvents(trajectories2.get(0), false,
                                                swerve, intake,
                                                elevator, wrist).until(
                                                                () -> swerve
                                                                                .getPitch() < -FieldDependentConstants.CurrentField.CHARGE_STATION_MAX_ANGLE),
                                Commands.parallel(
                                                new AlignWithGyro(swerve, true)
                                                                .andThen(new RunCommand(swerve::xPos, swerve)),
                                                IntakeToPosition.home(wrist, elevator)));
        }

        public static Command Score2Pos6(Wrist wrist, Elevator elevator, Intake intake, Swerve swerve) {
                List<PathPlannerTrajectory> trajectories = PathPlanner.loadPathGroup("2Balance6",
                                new PathConstraints(3.1, 1.1));
                return Commands.sequence(
                                resetGyroAndPos(swerve, trajectories.get(0)),
                                ScoreCommandFactory.scoreConeNode(wrist, elevator, intake, () -> ScoreHight.HIGH),
                                SwerveFollowTrajectory.FollowTrajectoryWithEvents(trajectories.get(0), false,
                                                swerve, intake,
                                                elevator, wrist),
                                SwerveFollowTrajectory.FollowTrajectoryWithEvents(trajectories.get(1), false,
                                                swerve, intake,
                                                elevator, wrist),
                                ScoreCommandFactory.alignAndScoreCubeHigh(wrist, elevator, intake,
                                                swerve),
                                IntakeToPosition.home(wrist, elevator));
        }

        public static Command ScoreMobilityBallance(Wrist wrist, Elevator elevator, Intake intake, Swerve swerve) {
                List<PathPlannerTrajectory> trajectories = PathPlanner.loadPathGroup("Mobility",
                                new PathConstraints(1.2, 0.8));
                return Commands.sequence(
                                resetGyroAndPos(swerve, trajectories.get(0)),
                                ScoreCommandFactory.scoreConeNode(wrist, elevator, intake, () -> ScoreHight.HIGH),
                                IntakeToPosition.home(wrist, elevator).until(() -> elevator.getHeightMeters() < 0.02),
                                SwerveFollowTrajectory.FollowTrajectoryWithEvents(trajectories.get(0), false,
                                                swerve, intake,
                                                elevator, wrist),
                                SwerveFollowTrajectory.FollowTrajectoryWithEvents(trajectories.get(1), false,
                                                swerve, intake,
                                                elevator, wrist)
                                                .until(() -> swerve
                                                                .getPitch() > FieldDependentConstants.CurrentField.CHARGE_STATION_MAX_ANGLE),
                                Commands.parallel(
                                                new AlignWithGyro(swerve, false)
                                                                .andThen(new RunCommand(swerve::xPos, swerve)),
                                                IntakeToPosition.home(wrist, elevator)));
        }

        public static Command ScoreMobility(Wrist wrist, Elevator elevator, Intake intake, Swerve swerve) {
                List<PathPlannerTrajectory> trajectories = PathPlanner.loadPathGroup("Mobility",
                                new PathConstraints(2, 1));
                return Commands.sequence(
                                resetGyroAndPos(swerve, trajectories.get(0)),
                                ScoreCommandFactory.scoreConeNode(wrist, elevator, intake, () -> ScoreHight.HIGH),
                                IntakeToPosition.home(wrist, elevator).until(() -> elevator.getHeightMeters() < 0.02),
                                SwerveFollowTrajectory.FollowTrajectoryWithEvents(trajectories.get(0), false,
                                                swerve, intake,
                                                elevator, wrist));
        }

        public static Command Score(Wrist wrist, Elevator elevator, Intake intake, Swerve swerve) {
                return Commands.sequence(
                                new InstantCommand(swerve::resetGyroTowardsDriverStation),
                                ScoreCommandFactory.scoreConeNode(wrist, elevator, intake, () -> ScoreHight.HIGH),
                                IntakeToPosition.home(wrist, elevator));
        }

        public static Command resetGyroAndPos(Swerve swerve, PathPlannerTrajectory trajectory) {
                return new InstantCommand(swerve::resetGyroTowardsDriverStation)
                                .andThen(() -> SwerveFollowTrajectory.resetPos(trajectory, swerve));
        }
        public static Command changeVisionState(Swerve swerve, PoseEstimationState state) {
                return new InstantCommand(() -> swerve.changeVisionState(state));
        }

}
