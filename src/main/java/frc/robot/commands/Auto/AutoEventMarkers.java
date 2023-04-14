// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.commands.objectManipulation.intake.IntakeCommandFactory;
import frc.robot.commands.objectManipulation.score.ScoreCommandFactory;
import frc.robot.commands.superStructure.IntakePositionCommandFactory;
import frc.robot.commands.superStructure.IntakeToPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

/** Add your docs here. */
public class AutoEventMarkers {

    public static Map<String, Command> getMap(Intake intakeAndShooter, Elevator elevator, Wrist wrist) {
        return Map.of(
            "IntakeCube", IntakeCommandFactory.intakeCube(wrist, elevator, intakeAndShooter),
            "ShootCube", ScoreCommandFactory.throwCube(wrist, elevator, intakeAndShooter).andThen(IntakeToPosition.home(wrist, elevator).withInterruptBehavior(InterruptionBehavior.kCancelSelf)),

            "IntakeStandingCone", IntakeToPosition.home(wrist, elevator).withInterruptBehavior(InterruptionBehavior.kCancelSelf),
            "IntakeToHigh", IntakePositionCommandFactory.IntakeToHighCubeNode(elevator, wrist).withInterruptBehavior(InterruptionBehavior.kCancelSelf),
            "IntakeToMid", IntakePositionCommandFactory.IntakeToMidCubeNode(elevator, wrist).withInterruptBehavior(InterruptionBehavior.kCancelSelf),
            "Home", IntakeToPosition.home(wrist, elevator).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
            );
    }
}
