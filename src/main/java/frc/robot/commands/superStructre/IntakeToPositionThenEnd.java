// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.superStructre;

import edu.wpi.first.wpilibj.Timer;
import frc.lib.util.ElevatorWristState;
import frc.robot.Constants.ElevatorWristStateConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Wrist;

/** Add your docs here. */
public class IntakeToPositionThenEnd extends IntakeToPosition {

    private boolean atGoal = false;

    public IntakeToPositionThenEnd(Wrist wrist, Elevator elevator, ElevatorWristState state) {
        super(wrist, elevator, state);
    }

    Timer setalTimer = new Timer();

    @Override
    public void execute() {
        super.execute();
        if (Math.abs(wrist.getAngleDeg() - goal.get().angle.getDegrees()) < ElevatorWristStateConstants.ANGLE_TOLERANCE
                && Math.abs(elevator.getHeight() - goal.get().height) < ElevatorWristStateConstants.HEIGHT_TOLERANCE) {
            setalTimer.start();
        } else {
            setalTimer.stop();
            setalTimer.reset();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}