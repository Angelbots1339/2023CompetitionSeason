// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.align;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Swerve;
import static frc.robot.Constants.PIDToPoseConstants.*;



public class AlignToTargetDresdon extends CommandBase {

  private Swerve swerve;
  private final Timer timer = new Timer();

  private Command trajCommand;
  private State idealState;
  private boolean isFinished = false;
  

  // Test Variables (Replace with the real ones when possible)
  private Translation2d testTagPos = new Translation2d(0, 0);
  private Rotation2d testTagRot = Rotation2d.fromDegrees(180); 



  /** Creates a new AlignToTarget. 
   * 
   * This command will align the robot to an apriltag target. The robot will follow a path, and then utilize PID when it gets close. 
   * 
   * @param swerve The swerve drivebase subsystem
  */
  public AlignToTargetDresdon(Swerve swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(swerve);

    this.timer.reset();
    this.timer.start();
    this.swerve = swerve;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    PathPlannerTrajectory traj = PathPlanner.generatePath(
    new PathConstraints(2, 2), 
    new PathPoint(new Translation2d(swerve.getPose().getX(), swerve.getPose().getY()), 
      Rotation2d.fromDegrees(Math.atan2(swerve.getCurrentVelocity().getX(), swerve.getCurrentVelocity().getY())), swerve.getYaw()), // position, heading(direction to move in), orientation
    new PathPoint( testTagPos, testTagRot, testTagRot) // position, heading(direction to move in), orientation  
);

int idealStateIdx = 0;
double idealStateDeltaX = 0;
double idealStateDeltaY = 0;


for(int i = traj.getStates().size() / 2; i < traj.getStates().size(); i++) {

  Translation2d currentTranslation = new Translation2d(traj.getState(i).poseMeters.getX(), traj.getState(i).poseMeters.getY());

  double xComponent = traj.getState(i).velocityMetersPerSecond / Math.cos(traj.getState(i).poseMeters.getRotation().getRadians());
  double yComponent = traj.getState(i).velocityMetersPerSecond / Math.sin(traj.getState(i).poseMeters.getRotation().getRadians());


  double deltaX = Math.abs(xComponent - Math.abs(currentTranslation.getX() - testTagPos.getX()) * PID_TO_POSE_X_P);
  double deltaY = Math.abs(yComponent - Math.abs(currentTranslation.getY() - testTagPos.getY()) * PID_TO_POSE_Y_P);

  if(deltaX <= idealStateDeltaX && deltaY <= idealStateDeltaY){ // TODO Find a better way to determine which state is better
    idealStateIdx = i;
    idealStateDeltaX = deltaX;
    idealStateDeltaY = deltaY;
  }
}

this.idealState = traj.getState(idealStateIdx);
this.trajCommand = swerve.followTrajectoryCommandCancelable(traj, this.idealState.timeSeconds);
this.trajCommand.schedule();
  }

  private Translation2d findIdealPoint() {
    return new Translation2d(0, 0); // TODO Find the closest point on a predefined arc in relation to the robot's current position
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() >= idealState.timeSeconds){
      isFinished = swerve.PIDToPose(new Pose2d(testTagPos, testTagRot));
    }
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    trajCommand.cancel();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isFinished;
  }
}

