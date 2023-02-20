package frc.robot;

import java.util.Map;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.DoubleTopic;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.math.Conversions;
import frc.lib.util.ElevatorWristState;
import frc.lib.util.logging.LoggedSubsystem;
import static frc.robot.Constants.ElevatorWristStateConstants.*;

import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.*;
import frc.robot.commands.align.AlignOnChargingStation;
import frc.robot.commands.align.AlignToConeNodeLimelightOnly;
import frc.robot.commands.align.AlignToPos;
import frc.robot.commands.auto.ExamplePathPlannerAuto;
import frc.robot.commands.objectManipulation.intake.IntakeCommandFactory;
import frc.robot.commands.superStructre.IntakeToPosition;
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

        private final LoggedSubsystem log = new LoggedSubsystem("RobotContainer", LoggingConstants.ROBOT_CONTAINER);
        /* Controllers */
        private final XboxController driver = new XboxController(0);
        private final Joystick test = new Joystick(1);
        /* Subsystems */
        private final Swerve swerve = new Swerve();
        private final Elevator elevator = new Elevator();
        private final Wrist wrist = new Wrist();
        private final IntakeAndShooter intakeAndShooter = new IntakeAndShooter();

        /* States */
        private boolean isAngularDrive = false;
        private boolean isFieldCentric = false;

        /* Subscribers */
        private GenericEntry PoseFinderElevator;
        private GenericEntry PoseFinderWrist;

        private GenericEntry shootPercent;
        private GenericEntry intakePercent;

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
        private final JoystickButton wristTest = new JoystickButton(driver,
                        XboxController.Button.kX.value);
        private final JoystickButton elevatorTest = new JoystickButton(driver,
                        XboxController.Button.kB.value);


        private final JoystickButton intakeUprightCone = new JoystickButton(driver,
                        XboxController.Button.kRightBumper.value);
        private final JoystickButton intakeCube = new JoystickButton(driver,
                        XboxController.Button.kLeftBumper.value);
        private final Trigger intakeFallenCone = intakeCube.and(intakeUprightCone);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                // swerve.setDefaultCommand(
                //                 new TeleopSwerve(swerve, translation, strafe, rotation, angle, () -> false,
                //                                 true // Is field relative
                //                 ));

                log.addDouble("Translation", (Supplier<Double>) translation, "Drive values");
                log.addDouble("Strafe", (Supplier<Double>) strafe, "Drive values");
                log.addDouble("Rotation", (Supplier<Double>) rotation, "Drive values");
                log.addDouble("Angle", () -> angle.get().getDegrees(), "Drive values");
                
                // Configure the button bindings
                
                
                elevator.setDefaultCommand(new IntakeToPosition(wrist, elevator, HOME));
                
                NetworkTable testing =  NetworkTableInstance.getDefault().getTable("Testing");

                ShuffleboardTab poseFinder = Shuffleboard.getTab("poseFinder");
                
                
                PoseFinderElevator = poseFinder.add("height", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 0, "max", 1.304079773806718)).getEntry();
                PoseFinderWrist = poseFinder.add("angle", 13).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 13, "max", 204.462891)).getEntry();

                shootPercent = poseFinder.add("shoot", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 1)).getEntry();
               
                intakePercent = poseFinder.add("intake", 0).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", -1, "max", 1)).getEntry();
              
                SmartDashboard.putNumber("testtt", ElevatorConstants.clicksToMeters(49220));
                
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
                zeroEncoders.onTrue(new InstantCommand(swerve::alignPoseNonVisionEstimator));

                //intakeCube.whileTrue(IntakeCommandFactory.intakeCube(wrist, elevator, intakeAndShooter));
                //intakeUprightCone.whileTrue(IntakeCommandFactory.intakeUprightCone(wrist, elevator, intakeAndShooter));
                //intakeFallenCone.whileTrue(IntakeCommandFactory.intakeFallenCone(wrist, elevator, intakeAndShooter));

                elevatorTest.whileTrue(new IntakeToPosition(wrist, elevator, () -> new ElevatorWristState(PoseFinderWrist.getDouble(13), PoseFinderElevator.getDouble(0))));
                wristTest.whileTrue(new StartEndCommand(() -> intakeAndShooter.runIntakeAtPercent(shootPercent.getDouble(0), intakePercent.getDouble(0)), () -> intakeAndShooter.disable(), intakeAndShooter));

        }

        /**
         * Use this to pass the autonomous command to the main {@link Robot} class.
         *
         * @return the command to run in autonomous
         */
        public Command getAutonomousCommand() {
                // An ExampleCommand will run in autonomous
                return new ExamplePathPlannerAuto(swerve);
        }

        public void resetToAbsloute() {
                swerve.resetToAbsolute();
        }

        public Runnable getSwerveBuffer() {
                return swerve.bufferYaw();
        }
 

}
