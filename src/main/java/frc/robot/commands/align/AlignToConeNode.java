// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.align;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.lib.LimeLight;
import frc.robot.subsystems.Swerve;

/** Add your docs here. */
public class AlignToConeNode extends CommandBase {
    /** Creates a new SimpleAlignToTarget. */
    Swerve swerve;
    // private ShuffleboardLayout translationLayout =
    // Shuffleboard.getTab("VisionTuning").getLayout("Translation",
    // BuiltInLayouts.kList);
    // private ShuffleboardLayout strafeLayout =
    // Shuffleboard.getTab("VisionTuning").getLayout("Strafe",
    // BuiltInLayouts.kList);

    private double translationKP = 0.0;
    private double strafeKP = 0.0;
    private double KS = 0.0;

    PIDController translationController = new PIDController(translationKP, 0, 0);
    PIDController strafeController = new PIDController(strafeKP, 0, 0);

  public AlignToConeNode(Swerve swerve) {
    this.swerve = swerve;
    addRequirements(swerve);
    //translationLayout.add(translationController);
    //strafeLayout.add(strafeController);

    // Use addRequirements() here to declare subsystem dependencies.
  }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        SmartDashboard.putBoolean("target", LimeLight.hasTargets());
        if (LimeLight.hasTargets()) {
       
            SmartDashboard.putNumber("targetPoseHorizontalOffset", LimeLight.getHorizontalOffset());
            SmartDashboard.putNumber("targetArea", LimeLight.getTargetArea());

            double strafe = strafeController.calculate(LimeLight.getHorizontalOffset(), 0)
                    + KS * Math.signum(strafeController.getPositionError());

            //TODO fix setpoint
            double translation = translationController.calculate(LimeLight.getTargetArea(), 1)
                    + KS * Math.signum(translationController.getPositionError());


            SmartDashboard.putNumber("strafe", strafe);
            SmartDashboard.putNumber("translation", translation);

           
            // swerve.angularDrive(new Translation2d(translation, strafe),
            // Rotation2d.fromDegrees(0), true, true);

        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}
