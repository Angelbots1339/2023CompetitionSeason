package frc.robot;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.math.Conversions;
import frc.lib.util.logging.LoggedSubsystem;
import static frc.robot.Constants.ElevatorWristStateConstants.*;

import frc.robot.commands.*;
import frc.robot.commands.align.AlignOnChargingStation;
import frc.robot.commands.auto.AutoFactory;
import frc.robot.commands.objectManipulation.intake.IntakeCommandFactory;
import frc.robot.commands.objectManipulation.score.ScoreCommandFactory;
import frc.robot.commands.superStructure.IntakePositionCommandFactory;
import frc.robot.commands.superStructure.IntakeToPosition;
import frc.robot.subsystems.*;
import frc.robot.vison.RetroReflectiveTargeter;

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

        private final LoggedSubsystem log = new LoggedSubsystem("RobotContainer", LoggingConstants.ROBOT_CONTAINER);
        /* Controllers */
        private final XboxController driver = new XboxController(0);
        private final Joystick test = new Joystick(1);
        /* Subsystems */
        private final Swerve swerve = new Swerve();
        private final Elevator elevator = new Elevator();
        private final Wrist wrist = new Wrist();
        private final Intake intake = new Intake();

        /* States */
        private boolean isAngularDrive = false;
        private boolean isFieldCentric = false;

        /* Subscribers */
        private GenericEntry PoseFinderElevator;
        private GenericEntry PoseFinderWrist;

        private GenericEntry shootPercent;
        private GenericEntry intakePercent;

        private SendableChooser<Command> autoChooser = new SendableChooser<Command>();

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
        private final Trigger zeroGyro = new JoystickButton(driver,
                        XboxController.Button.kStart.value);
        private final Trigger alignOnChargingStation = new JoystickButton(driver,
                        XboxController.Button.kBack.value);

        private final Trigger manualScoreHigh = new JoystickButton(driver,
                        XboxController.Button.kY.value);
        private final Trigger manualScoreMid = new JoystickButton(driver,
                        XboxController.Button.kA.value);
        private final Trigger autoScoreHigh = new JoystickButton(driver,
                        XboxController.Button.kX.value);
        private final Trigger autoScoreMid = new JoystickButton(driver,
                        XboxController.Button.kB.value);

        private final Trigger runIntakeGeneral = new JoystickButton(driver,
                        XboxController.Button.kRightBumper.value);
        private final Trigger runOuttakeGeneral = new JoystickButton(driver,
                        XboxController.Button.kLeftBumper.value);

        private final Trigger runOuttakeForHigh = runOuttakeGeneral.and(manualScoreHigh);
        private final Trigger runOuttakeForLow = runOuttakeGeneral.and(manualScoreMid);

        private final Trigger intakeToFallenCone = new Trigger(() -> driver.getLeftTriggerAxis() > 0.1);
        private final Trigger intakeToStandingCone = new Trigger(() -> driver.getRightTriggerAxis() > 0.1);

        private final Trigger runIntakeFallenCone = runIntakeGeneral.and(intakeToFallenCone);
        private final Trigger runIntakeStandingCone = runIntakeGeneral.and(intakeToStandingCone);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                swerve.setDefaultCommand(
                                new TeleopSwerve(swerve, translation, strafe, rotation, angle, () -> false,
                                                true // Is field relative
                                ));

                // Configure the button bindings
                // autoChooser.setDefaultOption("Test Auto", TestAutoFactory.getTestAuto(swerve,
                // elevator, wrist, intake));
                // autoChooser.setDefaultOption("Test Auto", TestAutoFactory.getTestAuto(swerve,
                // elevator, wrist, intake));x

                elevator.setDefaultCommand(new IntakeToPosition(wrist, elevator, HOME));

                NetworkTable testing = NetworkTableInstance.getDefault().getTable("Testing");

                ShuffleboardTab poseFinder = Shuffleboard.getTab("poseFinder");

                PoseFinderElevator = poseFinder.add("height", 0).withWidget(BuiltInWidgets.kNumberSlider)
                                .withProperties(Map.of("min", 0, "max", 1.304079773806718)).getEntry();
                PoseFinderWrist = poseFinder.add("angle", 13).withWidget(BuiltInWidgets.kNumberSlider)
                                .withProperties(Map.of("min", 13, "max", 204.462891)).getEntry();

                shootPercent = poseFinder.add("shoot", 0).withWidget(BuiltInWidgets.kNumberSlider)
                                .withProperties(Map.of("min", -1, "max", 1)).getEntry();

                intakePercent = poseFinder.add("intake", 0).withWidget(BuiltInWidgets.kNumberSlider)
                                .withProperties(Map.of("min", -1, "max", 1)).getEntry();

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
                alignOnChargingStation.whileTrue(new AlignOnChargingStation(swerve));

                intakeToStandingCone.whileTrue(IntakePositionCommandFactory.IntakeToStandingConeNode(elevator, wrist));
                intakeToFallenCone.whileTrue(IntakePositionCommandFactory.IntakeToFallenConeNode(elevator, wrist));

                manualScoreHigh.whileTrue(IntakePositionCommandFactory.IntakeToHigh(elevator, wrist, intake));
                manualScoreMid.whileTrue(IntakePositionCommandFactory.IntakeToMid(elevator, wrist, intake));

                autoScoreHigh.whileTrue(ScoreCommandFactory.alignAndScoreHigh(wrist, elevator, intake, swerve));
                autoScoreMid.whileTrue(ScoreCommandFactory.alignAndScoreMid(wrist, elevator, intake, swerve));

                runIntakeFallenCone.whileTrue(IntakeCommandFactory.runIntakeForFallenCone(intake));
                runIntakeStandingCone.whileTrue(IntakeCommandFactory.runIntakeForStandingCone(intake));

                runOuttakeForHigh.whileTrue(ScoreCommandFactory.outtakeHigh(intake));
                runOuttakeForLow.whileTrue(ScoreCommandFactory.outtakeMid(intake));

                runIntakeGeneral.whileTrue(new StartEndCommand(
                                () -> intake.runIntakeAtPercent(FieldDependentConstants.CurrentField.INTAKE_GENERAL),
                                intake::disable, intake));

                runOuttakeGeneral.whileTrue(new StartEndCommand(
                                () -> intake.runIntakeAtPercent(-FieldDependentConstants.CurrentField.OUTTAKE_GENERAL),
                                intake::disable, intake));

                // leftTrigger.whileTrue(new IntakeToPosition(wrist, elevator, () -> new
                // ElevatorWristState(PoseFinderWrist.getDouble(13),
                // PoseFinderElevator.getDouble(0))));
        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An ExampleCommand will run in autonomous
                return AutoFactory.Score2(wrist, elevator, intake, swerve); // AutoFactory.Score2(wrist, elevator,
                                                                            // intake, swerve);
        }

        public void resetToAbsloute() {
                swerve.resetToAbsolute();
        }

        public Runnable getSwerveBuffer() {
                return swerve.bufferYaw();
        }

        public double getLimelightOffset() {
                return RetroReflectiveTargeter.getYOffsetFromConeOffset(swerve.getPose(), intake.getConeOffset());
        }

}
