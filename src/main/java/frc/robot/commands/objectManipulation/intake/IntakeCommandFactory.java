// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.objectManipulation.intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.lib.util.ElevatorWristState;
import frc.robot.Constants.IntakeConstants;
import frc.robot.commands.superStructre.IntakeToPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeAndShooter;
import frc.robot.subsystems.Wrist;

/** Add your docs here. */
public class IntakeCommandFactory {
    public static Command intakeFallenCone(Wrist wrist, Elevator elevator, IntakeAndShooter intakeAndShooter) {
        return new IntakeToPosition(wrist, elevator, new ElevatorWristState(30, 0.5)).alongWith(
                new RunIntakeForFallenCone(intakeAndShooter));
    }

    public static Command intakeCube(Wrist wrist, Elevator elevator, IntakeAndShooter intakeAndShooter) {
        return new IntakeToPosition(wrist, elevator, new ElevatorWristState(40, 0.5))
                .alongWith(runIntakeForCube(intakeAndShooter));
    }

    public static Command intakeUprightCone(Wrist wrist, Elevator elevator, IntakeAndShooter intakeAndShooter) {
        return new IntakeToPosition(wrist, elevator, new ElevatorWristState(50, 0.5)).alongWith(
                new RunIntakeForUprightCone(intakeAndShooter));
    }

    public static Command intakeConeSubStation(Wrist wrist, Elevator elevator, IntakeAndShooter intakeAndShooter) {
        return new IntakeToPosition(wrist, elevator, new ElevatorWristState(0, 0)).alongWith(
                new RunIntakeForUprightCone(intakeAndShooter));
    }

    public static Command intakeCubeSubStation(Wrist wrist, Elevator elevator, IntakeAndShooter intakeAndShooter) {
        return new IntakeToPosition(wrist, elevator, new ElevatorWristState(0, 0)).alongWith(
                new RunIntakeForUprightCone(intakeAndShooter));
    }

    public static Command runIntakeForCube(IntakeAndShooter intakeAndShooter) {
        return new StartEndCommand(() -> intakeAndShooter.runIntakeAtPercent(IntakeConstants.INTAKE_CONE_SHOOT_PERCENT,
                IntakeConstants.INTAKE_CUBE_PERCENT), () -> intakeAndShooter.disable(), intakeAndShooter)
                .until(intakeAndShooter::cubeInRange);
    }

    public enum GroundIntakeType {
        FallenCone, Cube, UprightCone
    }
}
