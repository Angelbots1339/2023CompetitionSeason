package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopSwerve extends CommandBase {
  private Swerve swerve;
  private DoubleSupplier translation;
  private DoubleSupplier strafe;
  private DoubleSupplier rotation;
  private Supplier<Rotation2d> angle;
  private boolean isFieldRelative;
  private BooleanSupplier isAngularDrive;

  /**
   * @param swerve          subsystem
   * @param translation     -1 -> 1: 1 is forward -1 is backwards
   * @param strafe          -1 -> 1: 1 is right -1 is left
   * @param rotation        -1 -> 1: 1 is turn ccw -1 is turn cw
   * @param angle           0 -> 360: 0 is up ccw is + cw is - (for angular drive)
   * @param isAngularDrive
   * @param isFieldRelative
   */
  public TeleopSwerve(Swerve swerve, DoubleSupplier translation, DoubleSupplier strafe, DoubleSupplier rotation,
      Supplier<Rotation2d> angle, BooleanSupplier isAngularDrive, boolean isFieldRelative) {
    this.swerve = swerve;
    addRequirements(swerve);

    this.translation = translation;
    this.strafe = strafe;
    this.rotation = rotation;
    this.angle = angle;
    this.isFieldRelative = isFieldRelative;
    this.isAngularDrive = isAngularDrive;
  }

  @Override
  public void execute() {
    /* Drive */
    if (isAngularDrive.getAsBoolean()) {
      swerve.angularDrive(
          new Translation2d(translation.getAsDouble(), strafe.getAsDouble()).times(Constants.Swerve.MAX_SPEED),
          angle.get(),
          isFieldRelative, // Field relative
          true);
    } else {
      swerve.drive(
          new Translation2d(translation.getAsDouble(), strafe.getAsDouble()).times(Constants.Swerve.MAX_SPEED),
          (rotation.getAsDouble() * Constants.Swerve.MAX_ANGULAR_VELOCITY),
          isFieldRelative, // Field relative
          true);
    }
  }

}
