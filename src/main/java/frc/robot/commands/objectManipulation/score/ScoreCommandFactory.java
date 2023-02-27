// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.objectManipulation.score;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.robot.FieldDependentConstants;
import frc.robot.commands.align.AlignToAprilTag;
import frc.robot.commands.align.AlignToConeNodeLimelightOnly;
import frc.robot.commands.superStructure.IntakePositionCommandFactory;
import frc.robot.commands.superStructure.IntakeToPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.vision.RetroReflectiveTargeter;

/** Add your docs here. */
public class ScoreCommandFactory {

        public static Command alignAndScoreHigh(Wrist wrist, Elevator elevator, Intake intake,
                        Swerve swerve) {
                return new SelectCommand(Map.of(
                                IntakeState.CONE, alignAndScoreConeFavorHigh(wrist, elevator, intake, swerve),
                                IntakeState.CUBE, alignAndScoreCubeHigh(wrist, elevator, intake, swerve),
                                IntakeState.NONE, alignAndScoreConeFavorHigh(wrist, elevator, intake, swerve)

                ), intake::getState);
        }

        public static Command alignAndScoreMid(Wrist wrist, Elevator elevator, Intake intake,
                        Swerve swerve) {
                return new SelectCommand(Map.of(
                                IntakeState.CONE, alignAndScoreConeFavorMid(wrist, elevator, intake, swerve),
                                IntakeState.CUBE, alignAndScoreCubeMid(wrist, elevator, intake, swerve),
                                IntakeState.NONE, alignAndScoreConeFavorMid(wrist, elevator, intake, swerve)

                ), intake::getState);
        }

        public static Command alignAndScoreConeFavorHigh(Wrist wrist, Elevator elevator, Intake intake,
                        Swerve swerve) {
                return new AlignToConeNodeLimelightOnly(swerve, () -> intake.getConeOffset(), true).asProxy()
                                .andThen(scoreConeNode(wrist, elevator, intake,
                                                RetroReflectiveTargeter::getScoreHight));
        }

        public static Command alignAndScoreConeFavorMid(Wrist wrist, Elevator elevator, Intake intake,
                        Swerve swerve) {
                return new AlignToConeNodeLimelightOnly(swerve, () -> intake.getConeOffset(), false).asProxy()
                                .andThen(scoreConeNode(wrist, elevator, intake,
                                                RetroReflectiveTargeter::getScoreHight));
        }

        public static Command alignAndScoreCubeHigh(Wrist wrist, Elevator elevator, Intake intake,
                        Swerve swerve) {
                return new AlignToAprilTag(swerve).asProxy().andThen(scoreCubeNode(wrist, elevator, intake,
                                () -> ScoreHight.HIGH));
        }

        public static Command alignAndScoreCubeMid(Wrist wrist, Elevator elevator, Intake intake,
                        Swerve swerve) {
                return new AlignToAprilTag(swerve).asProxy().andThen(scoreCubeNode(wrist, elevator, intake,
                                () -> ScoreHight.MID));
        }

        public static Command scoreConeNode(Wrist wrist, Elevator elevator, Intake intake,
                        Supplier<Object> scoreHight) {
                return new SelectCommand(Map.of(
                                ScoreHight.HIGH,
                                extendThenPlace(wrist, elevator, intake,
                                                IntakePositionCommandFactory.IntakeToHighConeNode(elevator, wrist),
                                                FieldDependentConstants.CurrentField.HIGH_CONE_OUTTAKE_PERCENT),
                                ScoreHight.MID,
                                extendThenPlace(wrist, elevator, intake,
                                                IntakePositionCommandFactory.IntakeToMidConeNode(elevator, wrist),
                                                FieldDependentConstants.CurrentField.MID_CONE_OUTTAKE_PERCENT)),
                                scoreHight);
        }

        public static Command scoreCubeNode(Wrist wrist, Elevator elevator, Intake intake,
                        Supplier<Object> scoreHight) {
                return new SelectCommand(Map.of(
                                ScoreHight.HIGH, extendThenPlace(wrist, elevator, intake,
                                                IntakePositionCommandFactory.IntakeToHighCubeNode(elevator, wrist),
                                                FieldDependentConstants.CurrentField.HIGH_CUBE_OUTTAKE_PERCENT),
                                ScoreHight.MID,
                                extendThenPlace(wrist, elevator, intake,
                                                IntakePositionCommandFactory.IntakeToMidCubeNode(elevator, wrist),
                                                FieldDependentConstants.CurrentField.MID_CUBE_OUTTAKE_PERCENT)),
                                scoreHight);
        }

        public static Command extendThenPlace(Wrist wrist, Elevator elevator, Intake intake,
                        IntakeToPosition align, double outtakePercent) {
                return align.alongWith(outtakeConeAtSetPoint(intake, elevator, wrist, outtakePercent))
                                .until(() -> !intake.objectInIntake());
        }

        public static Command outtakeConeAtSetPoint(Intake intake, Elevator elevator, Wrist wrist,
                        double outtakePercent) {
                return new RunCommand(() -> {
                        if (!elevator.goalAtHome() && !wrist.goalAtHome() && elevator.atSetPoint()
                                        && wrist.atSetPoint()) {
                                intake.runIntakeAtPercent(-outtakePercent);
                        }

                }, intake).finallyDo((boolean end) -> intake.disable());
        }

        public static Command outtakeHigh(Intake intake) {
                return new SelectCommand(Map.of(
                                IntakeState.CONE,
                                outtakeAtPercent(intake,
                                                FieldDependentConstants.CurrentField.HIGH_CONE_OUTTAKE_PERCENT),
                                IntakeState.CUBE,
                                outtakeAtPercent(intake,
                                                FieldDependentConstants.CurrentField.HIGH_CUBE_OUTTAKE_PERCENT),
                                IntakeState.NONE,
                                outtakeAtPercent(intake,
                                                FieldDependentConstants.CurrentField.HIGH_CONE_OUTTAKE_PERCENT)),
                                intake::getState);
        }

        public static Command outtakeMid(Intake intake) {
                return new SelectCommand(Map.of(
                                IntakeState.CONE,
                                outtakeAtPercent(intake, FieldDependentConstants.CurrentField.MID_CONE_OUTTAKE_PERCENT),
                                IntakeState.CUBE,
                                outtakeAtPercent(intake, FieldDependentConstants.CurrentField.MID_CUBE_OUTTAKE_PERCENT),
                                IntakeState.NONE,
                                outtakeAtPercent(intake,
                                                FieldDependentConstants.CurrentField.MID_CONE_OUTTAKE_PERCENT)),
                                intake::getState);
        }

        public static Command outtakeAtPercent(Intake intake, double percent) {
                return new StartEndCommand(() -> intake.runIntakeAtPercent(percent), intake::disable, intake);
        }

        public enum ScoreHight {
                HIGH, MID, HYBRID
        }

}
