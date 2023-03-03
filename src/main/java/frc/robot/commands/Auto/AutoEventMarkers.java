// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.objectManipulation.intake.IntakeCommandFactory;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

/** Add your docs here. */
public class AutoEventMarkers {

    public static Map<String, Command> getMap(Intake intakeAndShooter, Elevator elevator, Wrist wrist) {
        return Map.of(
            "IntakeCube", IntakeCommandFactory.intakeCube(wrist, elevator, intakeAndShooter).withTimeout(5).asProxy(),
            "IntakeStandingCone", IntakeCommandFactory.intakeStandingCone(wrist, elevator, intakeAndShooter).withTimeout(5).asProxy()
            );
    }
}