package frc.robot;

import java.util.List;
import java.util.Map;
import java.util.function.Supplier;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.math.Conversions;
import frc.lib.util.Candle;
import frc.lib.util.ElevatorWristState;
import frc.lib.util.Candle.HumanPlayerCommStates;
import frc.lib.util.Candle.LEDState;
import frc.lib.util.logging.LoggedSubsystem;
import static frc.robot.Constants.ElevatorWristStateConstants.*;

import frc.robot.commands.*;
import frc.robot.commands.align.AlignOnChargingStation;
import frc.robot.commands.align.AlignWithGyro;
import frc.robot.commands.auto.AutoFactory;
import frc.robot.commands.auto.SwerveFollowTrajectory;
import frc.robot.commands.auto.TestAutoFactory;
import frc.robot.commands.objectManipulation.intake.IntakeCommandFactory;
import frc.robot.commands.objectManipulation.score.ScoreCommandFactory;
import frc.robot.commands.objectManipulation.score.ScoreCommandFactory.ScoreHight;
import frc.robot.commands.superStructure.IntakePositionCommandFactory;
import frc.robot.commands.superStructure.IntakeToPosition;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.vision.RetroReflectiveTargeter;

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
        private final XboxController test = new XboxController(1);
        /* Subsystems */
        private final Swerve swerve = new Swerve();
        private final Elevator elevator = new Elevator();
        private final Wrist wrist = new Wrist();
        private final Intake intake = new Intake();

        /* States */
        private boolean isAngularDrive = false;
        private boolean isFieldCentric = false;

        /* Subscribers */
        private GenericEntry isCone;

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
                        XboxController.Button.kA.value);
        private final Trigger manualScoreMid = new JoystickButton(driver,
                        XboxController.Button.kB.value);
        private final Trigger autoScoreHigh = new JoystickButton(driver,
                        XboxController.Button.kX.value);
        private final Trigger autoScoreMid = new JoystickButton(driver,
                        XboxController.Button.kY.value);

        private final Trigger runOuttakeGeneral = new JoystickButton(driver,
                        XboxController.Button.kRightBumper.value);
        private final Trigger runIntakeGeneral = new JoystickButton(driver,
                        XboxController.Button.kLeftBumper.value);

        private final Trigger runOuttakeForHigh = runOuttakeGeneral.and(manualScoreHigh);
        private final Trigger runOuttakeForLow = runOuttakeGeneral.and(manualScoreMid);

        private final Trigger intakeToStandingCone = new Trigger(() -> driver.getLeftTriggerAxis() > 0.1);
        private final Trigger intakeToFallenCone = new Trigger(() -> driver.getRightTriggerAxis() > 0.1);

        
        private final Trigger runIntakeFallenCone = runIntakeGeneral.and(intakeToFallenCone);
        private final Trigger runIntakeStandingCone = runIntakeGeneral.and(intakeToStandingCone);
        private final Trigger runOutakeStandingCone = runOuttakeGeneral.and(intakeToStandingCone);
        
        
        
        private final Trigger switchDeadSensorOverrideObject =  new JoystickButton(test,
        XboxController.Button.kX.value);
        private final Trigger resetSensors = new JoystickButton(test,
                        XboxController.Button.kB.value);
        private final Trigger signalCube = new JoystickButton(test,
                        XboxController.Button.kLeftBumper.value);
        private final Trigger signalCone = new JoystickButton(test,
                        XboxController.Button.kRightBumper.value);
        private final Trigger resetModules = new JoystickButton(test,
                        XboxController.Button.kRightBumper.value);

        private final Trigger dynamicHomeToggle = new JoystickButton(test,
                        XboxController.Button.kA.value);

                        
        private final Trigger distSensorOverrideLeft = new Trigger(() -> test.getPOV() > 0 && test.getPOV() < 180);
        private final Trigger distSensorOverrideRight = new Trigger(() -> test.getPOV() > 180);

        /**
         * The container for the robot. Contains subsystems, OI devices, and commands.
         */
        public RobotContainer() {
                swerve.setDefaultCommand(
                                new TeleopSwerve(swerve, translation, strafe, rotation, angle, () -> false,
                                                true // Is field relative
                                ));

                autoChooser.addOption("Score2BalancePos6",
                                AutoFactory.Score2BalancePos6(wrist, elevator, intake, swerve));
                autoChooser.addOption("Score2Pos6", AutoFactory.Score2Pos6(wrist, elevator, intake, swerve));
                autoChooser.addOption("ScoreBallance",
                                AutoFactory.ScoreBallanceWithGyro(wrist, elevator, intake, swerve));
                autoChooser.addOption("ScoreMobilityBallance",
                                AutoFactory.ScoreMobilityBallance(wrist, elevator, intake, swerve));
                autoChooser.addOption("ScoreMobility", AutoFactory.ScoreMobility(wrist, elevator, intake, swerve));
                autoChooser.addOption("ScoreGrabBallance", AutoFactory.ScoreGrabBallance(wrist, elevator, intake, swerve));
                autoChooser.addOption("ScoreGrabBallanceTurn", AutoFactory.ScoreGrabTurnBallance(wrist, elevator, intake, swerve));
                autoChooser.addOption("Score", AutoFactory.Score(wrist, elevator, intake, swerve));




                elevator.setDefaultCommand(new IntakeToPosition(wrist, elevator, HOME));

                SmartDashboard.putData("auto", autoChooser);

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
                zeroGyro.onTrue(new InstantCommand(swerve::resetGyroTowardsDriverStation));
                // alignOnChargingStation.whileTrue(new AlignOnChargingStation(swerve));

                intakeToStandingCone.whileTrue(IntakePositionCommandFactory.IntakeToStandingConeNode(elevator, wrist).withName("intakeToStandingCone"));
                intakeToFallenCone.whileTrue(IntakePositionCommandFactory.IntakeToFallenConeNode(elevator, wrist).withName("intakeToFallenCone"));

                manualScoreHigh.whileTrue(IntakePositionCommandFactory.IntakeToHigh(elevator, wrist, intake).withName("manualScoreHigh"));
                manualScoreMid.whileTrue(IntakePositionCommandFactory.IntakeToMid(elevator, wrist, intake).withName("manualScoreMid"));
                // manualScoreMid.whileTrue(new AlignOnChargingStation(swerve));

                autoScoreHigh.whileTrue(ScoreCommandFactory.alignAndScoreHigh(wrist, elevator, intake, swerve).withName("autoScoreHigh"));
                autoScoreMid.whileTrue(ScoreCommandFactory.alignAndScoreMid(wrist, elevator, intake, swerve).withName("autoScoreMid"));


                runIntakeFallenCone.whileTrue(IntakeCommandFactory.runIntakeForFallenCone(intake).withName("runIntakeFallenCone"));
                runIntakeStandingCone.whileTrue(IntakeCommandFactory.runIntakeForStandingCone(intake).withName("runIntakeStandingCone"));


                runOuttakeForHigh.whileTrue(ScoreCommandFactory.outtakeHigh(intake).withName("runOuttakeForHigh"));
                runOuttakeForLow.whileTrue(ScoreCommandFactory.outtakeMid(intake).withName("runOuttakeForLow"));

                runIntakeGeneral.whileTrue(new StartEndCommand(
                                () -> {
                                        intake.runIntakeAtPercent(FieldDependentConstants.CurrentField.INTAKE_GENERAL);
                                        Candle.getInstance().changeLedState(LEDState.Intake);
                                },
                                () -> {
                                        intake.disable();
                                        Candle.getInstance().changeLedState(LEDState.Idle);
                                }, intake).withName("runIntakeGeneral"));

                runOuttakeGeneral.whileTrue(new StartEndCommand(
                                () -> {
                                        intake.runIntakeAtPercent(-FieldDependentConstants.CurrentField.INTAKE_GENERAL);
                                        Candle.getInstance().changeLedState(LEDState.ReverseIntake);
                                },
                                () -> {
                                        intake.disable();
                                        Candle.getInstance().changeLedState(LEDState.Idle);
                                }, intake).withName("runOuttakeGeneral"));

                

                alignOnChargingStation.whileTrue(
                                new AlignWithGyro(swerve, false).andThen(new RunCommand(swerve::xPos, swerve)).withName("alignOnChargingStation"));

                // Operator Buttons
                switchDeadSensorOverrideObject.onTrue(new InstantCommand(() -> {
                        if (intake.getDeadSensorOverrideSate() == IntakeState.CONE)
                                intake.setDeadSensorOverrideSate(IntakeState.CUBE);
                        else
                                intake.setDeadSensorOverrideSate(IntakeState.CONE);
                }));
                resetSensors.onTrue(new InstantCommand(() -> intake.resetSensors()));

                signalCube.whileTrue(new StartEndCommand(
                                () -> {
                                        Candle.getInstance().changeLedState(LEDState.HumanPlayerCommunication);
                                        Candle.getInstance()
                                                        .changeHumanPlayerComState(HumanPlayerCommStates.SingleCube);
                                },
                                () -> Candle.getInstance().changeLedState(LEDState.Idle)));

                signalCone.whileTrue(new StartEndCommand(
                                () -> {
                                        Candle.getInstance().changeLedState(LEDState.HumanPlayerCommunication);
                                        Candle.getInstance()
                                                        .changeHumanPlayerComState(HumanPlayerCommStates.SingleCone);
                                },
                                () -> Candle.getInstance().changeLedState(LEDState.Idle)));

                

                distSensorOverrideRight.whileTrue(new StartEndCommand(intake::setDistSensorOverrideRight, intake::resetDistSensorOverride));
                distSensorOverrideLeft.whileTrue(new StartEndCommand(intake::setDistSensorOverrideLeft, intake::resetDistSensorOverride));


                resetModules.onTrue(new InstantCommand(() -> resetToAbsloute()));
                // manualScoreHigh.whileTrue(new IntakeToPosition(wrist, elevator, () -> new
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
                // return Commands.sequence(
                //         new InstantCommand(swerve::resetGyroTowardsDriverStation),
                //         ScoreCommandFactory.scoreConeNode(wrist, elevator, intake, () -> ScoreHight.HIGH),
                //         IntakeToPosition.home(wrist, elevator));
                return autoChooser.getSelected().withName("Auto"); // AutoFactory.Score2(wrist,

                // List<PathPlannerTrajectory> trajectories = PathPlanner.loadPathGroup("Mobility",
                //                 new PathConstraints(2, 1));

                // return AutoFactory.resetGyroAndPos(swerve, trajectories.get(0));
                                              
                // intake, swerve);
        }

        public void resetToAbsloute() {
                swerve.resetToAbsolute();
        }

        public Runnable getSwerveBuffer() {
                return swerve.bufferYaw();
        }

        public boolean getLimelightLeftOfTarget() {
                return RetroReflectiveTargeter.getYOffsetFromConeOffset(swerve.getPose(), intake
                                .getConeOffset()) < -FieldDependentConstants.CurrentField.LIMELIGHT_ALIGN_Y_TOLERANCE;
        }

        public boolean getLimelightRightOfTarget() {
                return RetroReflectiveTargeter.getYOffsetFromConeOffset(swerve.getPose(), intake
                                .getConeOffset()) > FieldDependentConstants.CurrentField.LIMELIGHT_ALIGN_Y_TOLERANCE;
        }
       

        // public void limeLightAlignAuto(){
        // RetroReflectiveTargeter.update(swerve.getPose(), true);
        // if(RetroReflectiveTargeter.getYOffsetFromConeOffset(swerve.getPose(),
        // intake.getConeOffset()) <
        // FieldDependentConstants.CurrentField.LIMELIGHT_ALIGN_Y_TOLERANCE){
        // Candle.getInstance().changeLedState();
        // };
        // }

}
