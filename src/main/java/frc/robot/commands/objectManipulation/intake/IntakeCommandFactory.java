// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.objectManipulation.intake;

import java.util.Map;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.lib.util.FieldUtil;
import frc.robot.FieldDependentConstants;
import frc.robot.commands.align.AlignToPos;
import frc.robot.commands.superStructure.IntakePositionCommandFactory;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

/** Add your docs here. */
public class IntakeCommandFactory {

    public static Command alignToSingleSubstation(Swerve swerve){
        return new AlignToPos(swerve, FieldUtil.getFirstAlignSingleSubstation(), 0.02, 0.05)
        .andThen(new AlignToPos(swerve,  FieldUtil.getSecondAlignSingleSubstation(), 0.05, 0.02));
    }
    public static Command alignToSingleSubstationAndIntake(Swerve swerve, Intake intake){
        return alignToSingleSubstation(swerve).andThen(new StartEndCommand(() -> intake.runIntakeAtPercent(0.5), () ->intake.disable(), intake)).until(intake::objectInIntake);
    }

    public static Command intakeFallenCone(Wrist wrist, Elevator elevator, Intake intake) {
        return new RunIntakeForCone(intake, FieldDependentConstants.CurrentField.INTAKE_FALLEN_CONE).deadlineWith(IntakePositionCommandFactory.IntakeToFallenConeNode(elevator, wrist));
    }

    public static Command intakeStandingCone(Wrist wrist, Elevator elevator, Intake intake) {
        return new RunIntakeForCone(intake, FieldDependentConstants.CurrentField.INTAKE_STANDING_CONE).deadlineWith(IntakePositionCommandFactory.IntakeToStandingConeNode(elevator, wrist));
    }
    public static Command intakeCube(Wrist wrist, Elevator elevator, Intake intake) {
        return runIntakeForCube(intake).deadlineWith(IntakePositionCommandFactory.IntakeToHighCubeNode(elevator, wrist));
    }

    public static Command runIntakeForStandingCone(Intake intake) {
        return new StartEndCommand(() -> intake.runIntakeAtPercent(
            FieldDependentConstants.CurrentField.INTAKE_STANDING_CONE) , intake::disable, intake);
    }

    public static Command runIntakeForFallenCone(Intake intake) {
        return new StartEndCommand(() -> intake.runIntakeAtPercent(
            FieldDependentConstants.CurrentField.INTAKE_FALLEN_CONE) , intake::disable, intake);
    }
    public static Command poseFinderIntake(Intake intake, String name) {
        
        ShuffleboardTab poseFinder = Shuffleboard.getTab("poseFinder");

        GenericEntry poseFinderHeightBeforeStartAngle = poseFinder.add("percent: " + name, 0)
                        .withWidget(BuiltInWidgets.kNumberSlider)
                        .withProperties(Map.of("min", 0, "max", 1, "Block increment", 0.05)).getEntry();
                        
        return new StartEndCommand(() -> intake.runIntakeAtPercent(
            poseFinderHeightBeforeStartAngle.getDouble(0)) , intake::disable, intake);
    }




    

    public static Command runIntakeForCube(Intake intake) {
        return new StartEndCommand(() -> intake.runIntakeAtPercent(
                FieldDependentConstants.CurrentField.INTAKE_CUBE), () -> intake.disable(), intake)
                .until(intake::cubeInIntake);
    }

    public enum GroundIntakeType {
        FallenCone, Cube, UprightCone
    }
}
