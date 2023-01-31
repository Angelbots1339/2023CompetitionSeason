package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.lib.math.Conversions;
import frc.lib.util.logging.LoggedSubsystem;
import frc.robot.commands.*;
import frc.robot.commands.Auto.examplePathPlannerAuto;
import frc.robot.commands.align.AlignToAprilTag;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

        private final LoggedSubsystem log = new LoggedSubsystem("RobotContainer", LoggingConstants.ROBOT_CONTAINER );
    /* Controllers */
    private final XboxController driver = new XboxController(0);
    /* Subsystems */
    private final Swerve swerve = new Swerve();


    /* States */
    private boolean isAngularDrive = false;

    /*----Controls----*/
    /* Drive Controls */
    // Up is positive right is positive

    // Maps to rect than applys deadband
    private Supplier<Double> translation = () -> MathUtil.applyDeadband(Conversions.mapJoystick(-driver.getLeftY(),
            -driver.getLeftX()), Constants.stickDeadband);
    private Supplier<Double> strafe = () -> MathUtil.applyDeadband(Conversions.mapJoystick(-driver.getLeftX(),
            -driver.getLeftY()), Constants.stickDeadband);
    private Supplier<Double> rotation = () -> MathUtil.applyDeadband(-driver.getRightX(), Constants.stickDeadband);

    private Rotation2d previousAngle = swerve.getYaw();
    private Supplier<Rotation2d> angle = () -> {
        if (Math.abs(driver.getRightX()) > Constants.angularStickDeadband &&
                Math.abs(driver.getRightX()) > Constants.angularStickDeadband) {
            previousAngle = Conversions.ConvertJoystickToAngle(driver.getRightX(),
                    driver.getRightY());
        }
        return previousAngle;
    };

    /* Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver,
            XboxController.Button.kY.value);
    private final JoystickButton zeroEncoders = new JoystickButton(driver,
            XboxController.Button.kA.value);
    private final JoystickButton switchDriveMode = new JoystickButton(driver,
            XboxController.Button.kRightStick.value);
    private final JoystickButton alignToTarget = new JoystickButton(driver,
            XboxController.Button.kB.value);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        swerve.setDefaultCommand(
                new TeleopSwerve(swerve, translation, strafe, rotation, angle, () -> false,
                        true // Is field relative
                ));

        log.addDouble("Translation", (Supplier<Double>)translation, "Drive values");
        log.addDouble("Strafe", (Supplier<Double>)strafe, "Drive values");
        log.addDouble("Rotation", (Supplier<Double>)rotation, "Drive values");
        log.addDouble("Angle", () -> angle.get().getDegrees(), "Drive values");

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */

        zeroGyro.onTrue(new InstantCommand(swerve::zeroGyro));
        zeroEncoders.onTrue(new InstantCommand(swerve::resetToVision));
        switchDriveMode.onTrue(new InstantCommand(() -> isAngularDrive = !isAngularDrive));
        alignToTarget.whileTrue(new AlignToAprilTag(swerve));

    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new examplePathPlannerAuto(swerve);
    }

    public void resetToAbsloute() {
        swerve.resetToAbsolute();
    }
}
