// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import frc.robot.commands.objectManipulation.intake.IntakeCommandFactory;
import frc.robot.commands.superStructure.IntakeToPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

/** Add your docs here. */
public class AutoEventMarkers {

    public static Map<String, Command> getMap(Intake intakeAndShooter, Elevator elevator, Wrist wrist) {
        return Map.of(
            "IntakeCube", IntakeToPosition.home(wrist, elevator).withInterruptBehavior(InterruptionBehavior.kCancelSelf),
            // /"IntakeStandingCone", IntakeCommandFactory.intakeStandingCone(wrist, elevator, intakeAndShooter).withTimeout(5).asProxy(),
            "IntakeStandingCone", IntakeToPosition.home(wrist, elevator).withInterruptBehavior(InterruptionBehavior.kCancelSelf),
            "Home", IntakeToPosition.home(wrist, elevator).withInterruptBehavior(InterruptionBehavior.kCancelSelf)
            );
    }
}
