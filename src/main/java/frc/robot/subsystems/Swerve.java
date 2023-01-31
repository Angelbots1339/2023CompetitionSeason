package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import org.photonvision.PhotonCamera;
import org.photonvision.RobotPoseEstimator;

import com.ctre.phoenix.sensors.Pigeon2;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import frc.robot.SwerveModule;
import frc.robot.commands.CancelableSwerveController;
import frc.robot.commands.Auto.SwerveFollowTrajectory;
import frc.lib.team254.util.TalonUtil;
import frc.lib.util.logging.LoggedSubsystem;
import frc.lib.util.logging.loggedObjects.LoggedFalcon;
import frc.lib.util.logging.loggedObjects.LoggedPigeon2;
import frc.lib.util.logging.loggedObjects.LoggedSwerveModule;
import frc.robot.LoggingConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.AutoConstants.*;
import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.VisionConstants.*;
import static frc.robot.Constants.PIDToPoseConstants.*;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public LoggedSubsystem logger;
    private PIDController angularDrivePID;
    private SwerveDrivePoseEstimator poseEstimator;
    private RobotPoseEstimator apriltagPoseEstimator;
    private AprilTagFieldLayout layout;

    private double lastTimePeriodic;
    private double lastTimeDrive;
    private double lastDesiredDegrees;

    private PIDController pidToPoseXController;
    private PIDController pidToPoseYController;

    private Translation2d currentVel = new Translation2d();
    private Pose2d lastPose = new Pose2d();

    private Field2d field2d = new Field2d();

    /**
     * Constructs a new Swerve subsystem
     */
    public Swerve() {
        logger = new LoggedSubsystem("Swerve", LoggingConstants.SWERVE);

        angularDrivePID = new PIDController(AngularDriveConstants.ANGLE_KP,
                AngularDriveConstants.ANGLE_KI, AngularDriveConstants.ANGLE_KD);
        angularDrivePID.enableContinuousInput(0, 360);

        angularDrivePID.setTolerance(AngularDriveConstants.TURN_TO_ANGLE_TOLERANCE);

        pidToPoseXController = new PIDController(PID_TO_POSE_X_P, PID_TO_POSE_X_I, PID_TO_POSE_X_D);
        pidToPoseYController = new PIDController(PID_TO_POSE_Y_P, PID_TO_POSE_Y_I, PID_TO_POSE_Y_D);
        pidToPoseXController.setTolerance(PID_TO_POSE_TOLERANCE);
        pidToPoseYController.setTolerance(PID_TO_POSE_TOLERANCE);

        try {
            layout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            e.printStackTrace();
        }

        mSwerveMods = new SwerveModule[] {
                new SwerveModule(0, Mod0.constants),
                new SwerveModule(1, Mod1.constants),
                new SwerveModule(2, Mod2.constants),
                new SwerveModule(3, Mod3.constants)
        };
        gyro = new Pigeon2(PIGEON_ID);
        TalonUtil.checkError(gyro.configFactoryDefault(), "Failed to config factory default on pigeon");
        zeroGyro();

        swerveOdometry = new SwerveDriveOdometry(KINEMATICS, getYaw(), getPositions());
        poseEstimator = new SwerveDrivePoseEstimator(KINEMATICS, getYaw(), getPositions(), new Pose2d(), STATE_STD_DEVS,
                VISION_MEASUREMENT_STD_DEVS);
        List<Pair<PhotonCamera, Transform3d>> camList = new ArrayList<Pair<PhotonCamera, Transform3d>>();
        camList.add(new Pair<PhotonCamera, Transform3d>(APRILTAG_CAM, APRILTAG_CAM_POS));
        
        apriltagPoseEstimator = new RobotPoseEstimator(layout, APRILTAG_POSE_STRATEGY, camList);

        initializeLog();

    }

    /**
     * 
     * @param translation   Translation2d holding the desired velocities on each
     *                      axis
     * @param rotation      Desired rotational velocity
     * @param fieldRelative If true, the robot behaves as field relative
     * @param isOpenLoop
     */
    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {

        ChassisSpeeds chassisSpeeds;
        if (fieldRelative) {
            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation,
                    getYaw());
        } else {
            chassisSpeeds = new ChassisSpeeds(
                    translation.getX(),
                    translation.getY(),
                    rotation);
        }
        chassisSpeeds = reduceSkewFromChassisSpeeds254(chassisSpeeds);
        logger.updateDouble("x", chassisSpeeds.vxMetersPerSecond, "Drive");
        logger.updateDouble("y", chassisSpeeds.vyMetersPerSecond, "Drive");
        logger.updateDouble("rot", chassisSpeeds.omegaRadiansPerSecond, "Drive");

        SwerveModuleState[] swerveModuleStates = KINEMATICS.toSwerveModuleStates(chassisSpeeds);

        setModuleStates(swerveModuleStates, isOpenLoop);
    }

    /**
     * @param translation    Translation2d holding the desired velocities on each
     *                       axis
     * @param desiredDegrees The desired orientation of the robot, whith 0 being
     *                       straight forward
     * @param fieldRelative  If true, the robot behaves as field relative
     * @param isOpenLoop
     */
    public void angularDrive(Translation2d translation, Rotation2d desiredDegrees, boolean fieldRelative,
            boolean isOpenLoop) {

        Rotation2d desiredAngularVelocity = Rotation2d.fromDegrees(
                (lastDesiredDegrees - desiredDegrees.getDegrees()) / (Timer.getFPGATimestamp() - lastTimeDrive));

        lastDesiredDegrees = desiredDegrees.getDegrees();
        lastTimeDrive = Timer.getFPGATimestamp();

        double rotation = 0;
        // convert yaw into -180 -> 180 and absolute
        double yaw = (0 > getYaw().getDegrees() ? getYaw().getDegrees() % 360 + 360 : getYaw().getDegrees() % 360);

        logger.updateDouble("desiredHeading", desiredDegrees.getDegrees(),
                "AngularDrive");
        logger.updateDouble("currentHeading", yaw, "AngularDrive");
        double pid = angularDrivePID.calculate(yaw, desiredDegrees.getDegrees());
        if (Math.abs(angularDrivePID.getPositionError()) >= 2) {
            rotation = MathUtil.clamp(pid + (-desiredAngularVelocity.getRadians() * AngularDriveConstants.ANGLE_KV) + // Kv
                                                                                                                      // //
                                                                                                                      // Velocity
            // Feedfoward
                    ((Math.signum(angularDrivePID.getPositionError())
                            * AngularDriveConstants.ANGLE_KS)) // Ks Static Friction Feedforward
                    , -MAX_ANGULAR_VELOCITY, MAX_ANGULAR_VELOCITY);

        }

        logger.updateDouble("Output rotation", rotation, "AngularDrive");
        logger.updateBoolean("At setpoint", angularDrivePID.atSetpoint(),
                "AngularDrive");

        drive(translation, rotation, fieldRelative, isOpenLoop);
    }
    /*-----Possible Solutions for Swerve Drive Skew------ */

    // See
    // https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/5?u=ethan1
    public ChassisSpeeds reduceSkewFromChassisSpeeds254(ChassisSpeeds chassisSpeeds) {
        frc.lib.team254.geometry.Pose2d robot_pose_vel = new frc.lib.team254.geometry.Pose2d(
                chassisSpeeds.vxMetersPerSecond * LOOPER_DT,
                chassisSpeeds.vyMetersPerSecond * LOOPER_DT,
                frc.lib.team254.geometry.Rotation2d.fromRadians(chassisSpeeds.omegaRadiansPerSecond * LOOPER_DT));
        frc.lib.team254.geometry.Twist2d twist_vel = frc.lib.team254.geometry.Pose2d.log(robot_pose_vel);
        return new ChassisSpeeds(
                twist_vel.dx / LOOPER_DT, twist_vel.dy / LOOPER_DT, twist_vel.dtheta / LOOPER_DT);

    }

    // See
    // https://www.chiefdelphi.com/t/whitepaper-swerve-drive-skew-and-second-order-kinematics/416964/11?u=ethan1
    public ChassisSpeeds reduceSkewFromChassisSpeedsFudgeFactor(ChassisSpeeds chassisSpeeds) {
        double linearVelocity = Math.sqrt(Math.pow(chassisSpeeds.vxMetersPerSecond, 2))
                + Math.pow(chassisSpeeds.vyMetersPerSecond, 2);
        double fudgeFactor = chassisSpeeds.omegaRadiansPerSecond / linearVelocity * FUDGE_FACTOR_KP;
        double unitOrthX = chassisSpeeds.vyMetersPerSecond / linearVelocity; // might need to swith signs to get corect
                                                                             // sign of movment
        double unitOrthy = -chassisSpeeds.vxMetersPerSecond / linearVelocity;
        double fudgeOrthX = fudgeFactor * unitOrthX;
        double fudgeOrthY = fudgeFactor * unitOrthy;
        return new ChassisSpeeds(fudgeOrthX + chassisSpeeds.vxMetersPerSecond,
                fudgeOrthY + chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
    }

    public ChassisSpeeds reduceSkewFromChassisSpeedsSimpleFudgeFactor(ChassisSpeeds chassisSpeeds) {
        double linearVelocity = Math.sqrt(Math.pow(chassisSpeeds.vxMetersPerSecond, 2))
                + Math.pow(chassisSpeeds.vyMetersPerSecond, 2);

        double fudgeFactor = Math.signum(chassisSpeeds.omegaRadiansPerSecond) * FUDGE_FACTOR_SIMPLE_KP;
        double unitOrthX = chassisSpeeds.vyMetersPerSecond / linearVelocity; // might need to swith signs to get corect
                                                                             // sign of movment
        double unitOrthy = -chassisSpeeds.vxMetersPerSecond / linearVelocity;
        double fudgeOrthX = fudgeFactor * unitOrthX;
        double fudgeOrthY = fudgeFactor * unitOrthy;
        return new ChassisSpeeds(fudgeOrthX + chassisSpeeds.vxMetersPerSecond,
                fudgeOrthY + chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
    }

    /*-----Setters----- */
    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        setModuleStates(desiredStates, false);
    }

    public void setChassisSpeeds(ChassisSpeeds chassisSpeeds) {
        SwerveModuleState[] swerveModuleStates = KINEMATICS.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(swerveModuleStates, true);
    }

    public void setModuleStates(SwerveModuleState[] desiredStates, Boolean isOpenLoop) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_SPEED);
        for (SwerveModule mod : mSwerveMods) {
            mod.setDesiredState(desiredStates[mod.moduleNumber], isOpenLoop);
        }
    }

    public void resetToAbsolute() {
        for (SwerveModule mod : mSwerveMods) {
            mod.resetToAbsolute();
        }
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
        poseEstimator.resetPosition(getYaw(), getPositions(), pose);
    }
    public void resetToVision() {

        Optional<Pair<Pose2d, Double>> aprilTagEstimation = getApriltagEstimatedPose(getEstimatedPose());
        if (aprilTagEstimation.isPresent()) {
            swerveOdometry.resetPosition(getYaw(), getPositions(), aprilTagEstimation.get().getFirst());
            poseEstimator.resetPosition(getYaw(), getPositions(), aprilTagEstimation.get().getFirst());
        }
    }

    public void zeroGyro() {
        gyro.setYaw(0);
    }

    /*-----Getters----- */

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Optional<Pair<Pose2d, Double>> getApriltagEstimatedPose(Pose2d prevEstimatedRobotPose) {
        apriltagPoseEstimator.setReferencePose(prevEstimatedRobotPose);

        double currentTime = Timer.getFPGATimestamp();
        Optional<Pair<Pose3d, Double>> result = apriltagPoseEstimator.update();
        if (result.get().getFirst() != null) {
            return Optional.ofNullable(new Pair<Pose2d, Double>(result.get().getFirst().toPose2d(),
                    currentTime - (result.get().getSecond() / 1000)));
        } else {
            return Optional.ofNullable(null);
        }
    }

    /**
     * Get the current velocity of the robot
     * 
     * @return A Translation2d containing the current X and Y velocity
     */
    public Translation2d getCurrentVelocity() {
        return currentVel;
    }

    public double getCurrentTotalVelocity() {
        return Math.sqrt(Math.pow(currentVel.getX(), 2) + Math.pow(currentVel.getY(), 2));
    }

    public Rotation2d getHeading() {
        return Rotation2d.fromRadians(Math.atan2(MathUtil.applyDeadband(currentVel.getY(), HEADING_DEADBAND),
                MathUtil.applyDeadband(currentVel.getX(), HEADING_DEADBAND)));
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (SwerveModule mod : mSwerveMods) {
            states[mod.moduleNumber] = mod.getPosition();
        }
        return states;
    }

    public Rotation2d getYaw() {
        return (INVERT_GYRO) ? Rotation2d.fromDegrees(360 - gyro.getYaw()) : Rotation2d.fromDegrees(gyro.getYaw());
    }

    /**
     * Use this to have the robot move to a position. Ideally use when the robot
     * isn't very far away.
     * 
     * @param target The Pose for the robot to go to
     * @return Whether or not the robot has gotten to within tolerance on both
     *         translation and rotation
     */
    public boolean PIDToPose(Pose2d target) {

        logger.updateDouble("set x", target.getX(), "PidPose");
        logger.updateDouble("set y", target.getY(), "PidPose");

        Translation2d calculatedValues = new Translation2d(
                pidToPoseXController.calculate(getEstimatedPose().getTranslation().getX(), target.getX()),
                pidToPoseYController.calculate(getEstimatedPose().getTranslation().getY(), target.getY()));
        angularDrive(calculatedValues, target.getRotation(), true, true);
        
        logger.updateDouble("out x", calculatedValues.getX(), "PidPose");
        logger.updateDouble("out y", calculatedValues.getY(), "PidPose");

        return pidToPoseXController.atSetpoint() && pidToPoseYController.atSetpoint() && angularDrivePID.atSetpoint();
    }

    public void TestTrajectoryGeneration(PathPoint endpoint) {
        field2d.getObject("traj").setTrajectory(SwerveFollowTrajectory.SwerveGenerateTrajectoryToPoint(endpoint, this));
    }

    @Override
    public void periodic() {
        swerveOdometry.update(getYaw(), getPositions());
        poseEstimator.update(getYaw(), getPositions());


      
        apriltagPoseEstimator.setReferencePose(getEstimatedPose());

        double currentTime = Timer.getFPGATimestamp();
        Optional<Pair<Pose3d, Double>> result = apriltagPoseEstimator.update();

        if(result.isPresent()){
            if (result.get().getFirst() != null) {
                poseEstimator.addVisionMeasurement(result.get().getFirst().toPose2d(), currentTime - (result.get().getSecond() / 1000));
           } 
        }
        SmartDashboard.putBoolean("Has target", result.isPresent());
        
        field2d.setRobotPose(getEstimatedPose());
        SmartDashboard.putData("est feild", field2d);

        double deltaTime = Timer.getFPGATimestamp() - lastTimePeriodic;
        currentVel = new Translation2d(
                (getEstimatedPose().getX() - lastPose.getX()) / deltaTime,
                (getEstimatedPose().getY() - lastPose.getY()) / deltaTime);
        lastPose = getEstimatedPose();
        lastTimePeriodic = Timer.getFPGATimestamp();

    }

    public void initializeLog() {
        logger.add(new LoggedPigeon2("Gyro", logger, gyro, "Gyro"));

        logger.addDouble("x", () -> getPose().getX(), "Pose");
        logger.addDouble("y", () -> getPose().getY(), "Pose");
        logger.addDouble("est x", () -> getEstimatedPose().getX(), "Pose");
        logger.addDouble("est y", () -> getEstimatedPose().getY(), "Pose");

        logger.addDouble("Heading", () -> getHeading().getDegrees(), "Pose");
        logger.addDouble("x velocity", () -> getCurrentVelocity().getX(), "Pose");
        logger.addDouble("y velocity", () -> getCurrentVelocity().getY(), "Pose");

        for (int i = 0; i < mSwerveMods.length; i++) {
            logger.add(new LoggedFalcon("Angle Motor: " + i, logger, mSwerveMods[i].getAngleMotor(), "Motor", true));
            logger.add(new LoggedFalcon("Drive Motor: " + i, logger, mSwerveMods[i].getDriveMotor(), "Motor", true));
            logger.add(new LoggedSwerveModule("Module : " + i, logger, mSwerveMods[i], "Module", true));
        }

        logger.addDouble("Mod : 0 v", () -> mSwerveMods[0].getState().speedMetersPerSecond, "Drive");
        logger.addDouble("Mod : 1 v", () -> mSwerveMods[1].getState().speedMetersPerSecond, "Drive");
        logger.addDouble("Mod : 2 v", () -> mSwerveMods[2].getState().speedMetersPerSecond, "Drive");
        logger.addDouble("Mod : 3 v", () -> mSwerveMods[3].getState().speedMetersPerSecond, "Drive");
    }

    /**
     * 
     * Returns a new command that, when ran, will make the swerve drivebase drive
     * along the path
     * 
     * @param traj        The trajectory to follow
     * @param isFirstPath If set to true, the robot odometry will be reset
     * @return The Command that, when ran, will make the swerve drivebase drive
     *         along the path
     */
    public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean isFirstPath) {
        // This is just an example event map. It would be better to have a constant,
        // global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));

        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        this.resetOdometry(traj.getInitialHolonomicPose());
                    }
                }),
                new PPSwerveControllerCommand(
                        traj,
                        this::getEstimatedPose, // Pose supplier
                        KINEMATICS, // SwerveDriveKinematics
                        new PIDController(3.2023, 0, 0), // X controller. Tune these values for your robot. Leaving them
                                                         // 0 will only use feedforwards.
                        new PIDController(3.2023, 0, 0), // Y controller (usually the same values as X controller)
                        new PIDController(THETA_KP, 0, 0), // Rotation controller. Tune these values for your robot.
                                                           // Leaving them 0 will only use feedforwards.
                        this::setModuleStates, // Module states consumer // This argument is optional if you don't use
                                               // event markers
                        this // Requires this drive subsystem
                ));
    }

    /**
     * 
     * Returns a new command that, when ran, will make the swerve drivebase drive
     * along the path
     * 
     * @param traj The trajectory to follow
     * @return The Command that, when ran, will make the swerve drivebase drive
     *         along the path
     */
    public Command followTrajectoryCommand(PathPlannerTrajectory traj) {
        return followTrajectoryCommand(traj, false);
    }

    /**
     * 
     * Returns a new command that, when ran, will make the swerve drivebase drive
     * along the path until the given time is reached, where it will stop the path
     * early
     * 
     * @param traj        The trajectory to follow
     * @param isFirstPath If set to true, the robot odometry will be reset
     * @param time        The time after which the path will be stopped
     * @return The Command that, when ran, will make the swerve drivebase drive
     *         along the path
     */
    public Command followTrajectoryCommandCancelable(PathPlannerTrajectory traj, boolean isFirstPath, double time) {
        // This is just an example event map. It would be better to have a constant,
        // global event map
        // in your code that will be used by all path following commands.
        HashMap<String, Command> eventMap = new HashMap<>();
        eventMap.put("marker1", new PrintCommand("Passed marker 1"));

        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        this.resetOdometry(traj.getInitialHolonomicPose());
                    }
                }),
                new CancelableSwerveController(
                        time,
                        traj,
                        this::getPose, // Pose supplier
                        KINEMATICS, // SwerveDriveKinematics
                        new PIDController(3.2023, 0, 0), // X controller. Tune these values for your robot. Leaving them
                                                         // 0 will only use feedforwards.
                        new PIDController(3.2023, 0, 0), // Y controller (usually the same values as X controller)
                        new PIDController(THETA_KP, 0, 0), // Rotation controller. Tune these values for your robot.
                                                           // Leaving them 0 will only use feedforwards.
                        this::setModuleStates, // Module states consumer // This argument is optional if you don't use
                                               // event markers
                        this // Requires this drive subsystem
                ));
    }

    /**
     * 
     * Returns a new command that, when ran, will make the swerve drivebase drive
     * along the path until the given time is reached, where it will stop the path
     * early
     * 
     * @param traj The trajectory to follow
     * @param time The time after which the path will be stopped
     * @return The Command that, when ran, will make the swerve drivebase drive
     *         along the path
     */
    public Command followTrajectoryCommandCancelable(PathPlannerTrajectory traj, double time) {
        return followTrajectoryCommandCancelable(traj, false, time);
    }

}