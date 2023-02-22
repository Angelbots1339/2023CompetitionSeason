// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.objectManipulation.score;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import frc.lib.util.ElevatorWristState;
import frc.lib.util.LimeLight;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.align.AlignToConeNodeLimelightOnly;
import frc.robot.commands.align.ApproachConeNode;
import frc.robot.commands.superStructre.IntakeToPosition;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.IntakeAndShooter;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wrist;

/** Add your docs here. */
public class ScoreCommandFactory {

    public static Command scoreConeNodeLimelight(Swerve swerve, Wrist wrist, Elevator elevator,
            IntakeAndShooter intakeAndShooter) {
        return new AlignToConeNodeLimelightOnly(swerve).asProxy().andThen(
                new ApproachConeNode(swerve).alongWith(alignOpenCone(wrist, elevator, intakeAndShooter, swerve)))
                .andThen(outtakeConeGeneral(intakeAndShooter));
    }




    public static Command alignOpenCone(Wrist wrist, Elevator elevator, IntakeAndShooter intakeAndShooter,
            Swerve swerve) {
        double horizontalOffset = LimeLight.getHorizontalOffset();
        if (LimeLight.hasTargets()) {
            if (Math.abs(horizontalOffset - VisionConstants.LIMELIGHT_HIGH_CONE_NODE_HORIZONTAL_OFFSET) < Math
                    .abs(horizontalOffset - VisionConstants.LIMELIGHT_MID_CONE_NODE_HORIZONTAL_OFFSET)) {
                return scoreConeNodeHigh(wrist, elevator, intakeAndShooter, ScoreHight.HIGH);
            }
            return scoreConeNodeHigh(wrist, elevator, intakeAndShooter, ScoreHight.MID);
        }
        return IntakeToPosition.home(wrist, elevator);
    }

    public static Command scoreConeNodeHigh(Wrist wrist, Elevator elevator, IntakeAndShooter intakeAndShooter,
            ScoreHight scoreHight) {
        if (intakeAndShooter.isUprightConeInIntake()) {
            switch (scoreHight) {
                case HIGH:
                    return new IntakeToPosition(wrist, elevator, new ElevatorWristState(0, 0));
                case MID:
                    return new IntakeToPosition(wrist, elevator, new ElevatorWristState(0, 0));
                case HYBRID:
                    return new IntakeToPosition(wrist, elevator, new ElevatorWristState(0, 0));
            }
        } else if (intakeAndShooter.isFallenConeInIntake()) {
            switch (scoreHight) {
                case HIGH:
                    return new IntakeToPosition(wrist, elevator, new ElevatorWristState(0, 0));
                case MID:
                    return new IntakeToPosition(wrist, elevator, new ElevatorWristState(0, 0));
                case HYBRID:
                    return new IntakeToPosition(wrist, elevator, new ElevatorWristState(0, 0));
            }
        } else { // cube
            switch (scoreHight) {
                case HIGH:
                    return new IntakeToPosition(wrist, elevator, new ElevatorWristState(0, 0));
                case MID:
                    return new IntakeToPosition(wrist, elevator, new ElevatorWristState(0, 0));
                case HYBRID:
                    return new IntakeToPosition(wrist, elevator, new ElevatorWristState(0, 0));
            }
        }
        return IntakeToPosition.home(wrist, elevator);
    }

    public static Command outtakeConeGeneral(IntakeAndShooter intakeAndShooter) {
        return new StartEndCommand(() -> intakeAndShooter.runIntakeAtPercent(1), () -> intakeAndShooter.disable(),
                intakeAndShooter).until(intakeAndShooter::isConeInIntake).withName("Outtake Cone General");
    }

    public static Command outtakeCubeGeneral(IntakeAndShooter intakeAndShooter) {
        return new StartEndCommand(() -> intakeAndShooter.runIntakeAtPercent(-1), () -> intakeAndShooter.disable(),
                intakeAndShooter).until(intakeAndShooter::isCubeInIntake).withName("Outtake Cube General");
    }

    public enum ScoreHight {
        HIGH, MID, HYBRID
    }

}
