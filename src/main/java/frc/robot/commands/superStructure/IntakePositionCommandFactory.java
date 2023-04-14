// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superStructure;

import java.util.Map;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import frc.robot.FieldDependentConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;

/** Add your docs here. */
public class IntakePositionCommandFactory {

    public static Command IntakeToHigh(Elevator elevator, Wrist wrist, Intake intake) {
        return new SelectCommand(Map.of(
                Intake.IntakeState.CONE, IntakeToHighConeNode(elevator, wrist),
                Intake.IntakeState.CUBE, IntakeToHighCubeNode(elevator, wrist),
                Intake.IntakeState.NONE, IntakeToHighConeNode(elevator, wrist)), intake::getState);
    }

    public static Command IntakeToMid(Elevator elevator, Wrist wrist, Intake intake) {
        return new SelectCommand(Map.of(
                Intake.IntakeState.CONE, IntakeToMidConeNode(elevator, wrist),
                Intake.IntakeState.CUBE, IntakeToMidCubeNode(elevator, wrist),
                Intake.IntakeState.NONE, IntakeToMidConeNode(elevator, wrist)), intake::getState);
    }

  


    public static IntakeToPosition IntakeToHighConeNode(Elevator elevator, Wrist wrist) {
        return IntakeToPosition.reachHeightBeforeStartAngle(wrist, elevator,
                FieldDependentConstants.CurrentField.HIGH_CONE,
                FieldDependentConstants.CurrentField.HIGH_CONE_HIGHT_BEFORE_ANGLE);
    }

    public static IntakeToPosition IntakeToMidConeNode(Elevator elevator, Wrist wrist) {
        return IntakeToPosition.reachHeightBeforeStartAngle(wrist, elevator,
                FieldDependentConstants.CurrentField.MID_CONE,
                FieldDependentConstants.CurrentField.MID_CONE_HIGHT_BEFORE_ANGLE);
    }

    public static IntakeToPosition IntakeToHighCubeNode(Elevator elevator, Wrist wrist) {
        return IntakeToPosition.reachHeightBeforeStartAngle(wrist, elevator,
                FieldDependentConstants.CurrentField.HIGH_CUBE,
                FieldDependentConstants.CurrentField.HIGH_CUBE_HIGHT_BEFORE_ANGLE);
    }

    public static IntakeToPosition IntakeToMidCubeNode(Elevator elevator, Wrist wrist) {
        return IntakeToPosition.reachHeightBeforeStartAngle(wrist, elevator,
                FieldDependentConstants.CurrentField.MID_CUBE,
                FieldDependentConstants.CurrentField.MID_CUBE_HIGHT_BEFORE_ANGLE);
    }

    public static IntakeToPosition IntakeToStandingConeNode(Elevator elevator, Wrist wrist) {
        return new IntakeToPosition(wrist, elevator, FieldDependentConstants.CurrentField.STANDING_CONE);
    }

    public static IntakeToPosition IntakeToFallenConeNode(Elevator elevator, Wrist wrist) {
        return IntakeToPosition.poseFinder(wrist, elevator, FieldDependentConstants.CurrentField.FALLEN_CONE);
    }


    

}
